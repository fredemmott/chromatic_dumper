// This implements both FLASH_PROGRAM and SET_FLASH_CMD
//
// This is because one sets the commands 'template', the other reads it - so, shared state
module lk_cmd_flash_program_t(
    input wire clk,

    input  wire        rx_valid,
    input  wire [7:0]  rx_data,

    input  wire        SET_FLASH_CMD_enable,
    output reg         SET_FLASH_CMD_complete,

    input  wire        FLASH_PROGRAM_enable,
    output reg         FLASH_PROGRAM_complete,

    output reg         cart_req_valid,
    output reg  [15:0] cart_req_address,
    output reg  [7:0]  cart_req_data,
    output reg         cart_req_wait_for_status,
    input  wire        cart_ready,
    input  wire        cart_complete,

    input  wire [15:0] var_address,
    input  wire [11:0] var_transfer_size // we only support up to 2048
);

typedef struct {
    logic [15:0] address;
    logic [7:0]  data;
} command_t;
command_t commands [0:5];

reg rx_valid_r;
reg [7:0] rx_data_r;
always @(posedge clk) begin
    rx_valid_r <= rx_valid;
    rx_data_r <= rx_data;
end

///// <SET_FLASH_CMD> /////

// Header:
//
// byte [0]: command set (ignored, only supporting AMD for now)
//      [1]: flash method (ignored, always doing FLASH_METHOD_UNBUFFERED for now)
//
// Commands (always exactly 6):
//
// byte [0..3]: address (only lower 16 bits for us on DMG)
//      [4..5]: data    (only lower 8 bits)
//
reg [2:0] SFC_cmd_idx;
reg [2:0] SFC_byte_idx;

typedef enum {
    SFC_RX_COMMAND_SET,
    SFC_RX_FLASH_METHOD,
    SFC_RX_COMMAND,
    SFC_COMPLETE
} SFC_state_t;
SFC_state_t SFC_state;
always @(posedge clk) SET_FLASH_CMD_complete <= SFC_state == SFC_COMPLETE;

wire SFC_rx_this_command_complete = rx_valid_r && (SFC_byte_idx == 5);
wire SFC_rx_all_commands_complete = SFC_rx_this_command_complete && (SFC_cmd_idx == 5);

SFC_state_t SFC_state_next;
always @(*) begin
    SFC_state_next = SFC_state;
    unique case (SFC_state)
        SFC_RX_COMMAND_SET: if (rx_valid_r) SFC_state_next = SFC_RX_FLASH_METHOD;
        SFC_RX_FLASH_METHOD: if (rx_valid_r) SFC_state_next = SFC_RX_COMMAND;
        SFC_RX_COMMAND: if (SFC_rx_all_commands_complete) SFC_state_next = SFC_COMPLETE;
        // SFC_COMPLETE: /* terminal until !enabled */ ;
        default: ;
    endcase
end
always @(posedge clk) begin
    if (!SET_FLASH_CMD_enable) begin
        SFC_state <= SFC_RX_COMMAND_SET;
    end else begin
        SFC_state <= SFC_state_next;
    end
end

always @(posedge clk) begin
    if (SFC_state != SFC_RX_COMMAND) begin
        SFC_byte_idx <= '0;
    end else if (rx_valid_r) begin
        SFC_byte_idx <= SFC_byte_idx + 3'd1;
    end
end

always @(posedge clk) begin
    if (SFC_state != SFC_RX_COMMAND) begin
        SFC_cmd_idx <= '0;
    end else if (SFC_rx_this_command_complete) begin
        SFC_cmd_idx <= SFC_cmd_idx + 1'd1;
    end
end

wire SFC_rx_command_byte = rx_valid_r && (SFC_state == SFC_RX_COMMAND);
reg [15:0] SFC_address;

always @(posedge clk) begin
    if (SFC_rx_command_byte) begin
        unique case (SFC_byte_idx)
            // 0, 1: /* unused, AGB-only */ ;
            2: SFC_address[15:8] <= rx_data_r;
            3: SFC_address[7:0] <= rx_data_r;
            // 4: /* unused, AGB-only */ ;
            5: commands[SFC_cmd_idx] <= '{SFC_address, rx_data_r};
            default: ;
        endcase
    end
end

///// </SET_FLASH_CMD> /////
///// <FLASH_PROGRAM> /////

// We're occupying a BRAM slot, we might as well fill it (well, close enough) :)
reg [7:0] FP_program [0:2047];
reg [11:0] FP_rx_count;
reg [15:0] FP_payload_address;
reg [7:0] FP_payload_data;
reg [11:0] FP_write_count;
reg [11:0] FP_write_count_next;
logic [2:0] FP_cmd_idx;
logic [2:0] FP_cmd_idx_next;
// up to 6 commands per data byte, up to 2048 data bytes, so up to 12288 commands
reg [13:0] FP_pending_cart_commands;

wire FP_rx_complete = (FP_rx_count == var_transfer_size);
wire FP_exec_complete = (FP_write_count == FP_rx_count);
wire FP_wait_cart_complete = (FP_pending_cart_commands == 0);

typedef enum {
    FP_RX_AND_EXEC,
    FP_EXEC_ONLY,
    FP_WAIT_CART_COMPLETE,
    FP_COMPLETE
} FP_state_t;
FP_state_t FP_state;
always @(posedge clk) FLASH_PROGRAM_complete <= (FP_state == FP_COMPLETE);

FP_state_t FP_state_next;
always @(*) begin
    FP_state_next = FP_state;
    unique case (FP_state)
        FP_RX_AND_EXEC: if (FP_rx_complete) FP_state_next = FP_EXEC_ONLY;
        FP_EXEC_ONLY: if (FP_exec_complete) FP_state_next = FP_WAIT_CART_COMPLETE;
        FP_WAIT_CART_COMPLETE: if (FP_wait_cart_complete) FP_state_next = FP_COMPLETE;
        // FP_COMPLETE: /* terminal until !enable */ ;
        default: ;
    endcase
end
always @(posedge clk) begin
    if (!FLASH_PROGRAM_enable) begin
        FP_state <= FP_RX_AND_EXEC;
    end else begin
        FP_state <= FP_state_next;
    end
end

always @(posedge clk) begin
    if (!FLASH_PROGRAM_enable) begin
        FP_rx_count <= 1'd0;
    end else if (rx_valid_r) begin
        FP_rx_count <= FP_rx_count + 1'd1;
    end
end

wire FP_rx = (FP_state == FP_RX_AND_EXEC) && rx_valid_r;
always @(posedge clk) begin
    if (FP_rx) begin
        FP_program[FP_rx_count] <= rx_data_r;
    end
end

command_t FP_cmd_it;
assign FP_cmd_it = commands[FP_cmd_idx];
wire FP_cmd_is_payload = (FP_cmd_it == '{default: 0});

wire FP_exec = cart_ready && !FP_exec_complete;
logic [15:0] FP_cart_address_next;
logic [7:0]  FP_cart_data_next;
always @(*) begin
    FP_cart_address_next = commands[FP_cmd_idx].address;
    FP_cart_data_next = commands[FP_cmd_idx].data;
    if (FP_cmd_is_payload) begin
        FP_cart_address_next = FP_payload_address;
        FP_cart_data_next = FP_payload_data;
    end
end
always @(posedge clk) begin
    cart_req_valid <= 1'b0;
    cart_req_address <= 16'd0;
    cart_req_data <= 8'd0;
    cart_req_wait_for_status <= 1'b0;
    if (FP_exec) begin
        cart_req_valid <= 1'd1;
        cart_req_address <= FP_cart_address_next;
        cart_req_data <= FP_cart_data_next;
        cart_req_wait_for_status <= FP_cmd_is_payload;
    end
end

always @(posedge clk) begin
    if (!FLASH_PROGRAM_enable) begin
        FP_pending_cart_commands <= '0;
    end else begin
        unique case ({FP_exec, cart_complete})
            2'b11, 2'b00: /* no change */ ;
            2'b10: FP_pending_cart_commands <= FP_pending_cart_commands + 14'd1;
            2'b01: FP_pending_cart_commands <= FP_pending_cart_commands - 14'd1;
        endcase
    end
end

always @(posedge clk) begin
    if (!FLASH_PROGRAM_enable) begin
        FP_payload_address <= var_address;
    end else if (FP_exec && FP_cmd_is_payload) begin
        FP_payload_address <= FP_payload_address + 16'd1;
    end
end

// In theory this is a cycle late
// In practice, we never need it for the first command
always @(posedge clk) FP_payload_data <= FP_program[FP_write_count];

always @(*) begin
    FP_cmd_idx_next = FP_cmd_idx;
    if (FP_write_count == FP_rx_count) begin
        FP_cmd_idx_next = 0;
    end else if (FP_exec) begin
        FP_cmd_idx_next = FP_cmd_is_payload ? '0 : (FP_cmd_idx + 3'd1);
    end
end
always @(posedge clk) begin
    if (!FLASH_PROGRAM_enable) FP_cmd_idx <= '0;
    else FP_cmd_idx <= FP_cmd_idx_next;
end

always @(*) begin
    FP_write_count_next = FP_write_count;
    if (FP_exec && FP_cmd_is_payload) FP_write_count_next = FP_write_count + 12'd1;
end
always @(posedge clk) begin
    if (!FLASH_PROGRAM_enable) FP_write_count <= 1'd0;
    else FP_write_count <= FP_write_count_next;
end
///// </FLASH_PROGRAM> /////

endmodule