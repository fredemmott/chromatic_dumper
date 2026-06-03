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
reg [5:0] SFC_byte_sr;

typedef enum {
    SFC_RX_COMMAND_SET,
    SFC_RX_FLASH_METHOD,
    SFC_RX_FLASH_WE_PIN, // TODO? shouldn't this be done by SET_VARIABLE?
    SFC_RX_COMMAND_0,
    SFC_RX_COMMAND_1,
    SFC_RX_COMMAND_2,
    SFC_RX_COMMAND_3,
    SFC_RX_COMMAND_4,
    SFC_RX_COMMAND_5,
    SFC_COMPLETE
} SFC_state_t;
SFC_state_t SFC_state;
always @(posedge clk) SET_FLASH_CMD_complete <= SFC_state == SFC_COMPLETE;

wire SFC_rx_this_command_complete = rx_valid_r && SFC_byte_sr[5];

SFC_state_t SFC_state_next;
always @(*) begin
    SFC_state_next = SFC_state;
    unique case (SFC_state)
        SFC_RX_COMMAND_SET: if (rx_valid_r) SFC_state_next = SFC_RX_FLASH_METHOD;
        SFC_RX_FLASH_METHOD: if (rx_valid_r) SFC_state_next = SFC_RX_FLASH_WE_PIN;
        SFC_RX_FLASH_WE_PIN: if (rx_valid_r) SFC_state_next = SFC_RX_COMMAND_0;
        SFC_RX_COMMAND_0: if (SFC_rx_this_command_complete) SFC_state_next = SFC_RX_COMMAND_1;
        SFC_RX_COMMAND_1: if (SFC_rx_this_command_complete) SFC_state_next = SFC_RX_COMMAND_2;
        SFC_RX_COMMAND_2: if (SFC_rx_this_command_complete) SFC_state_next = SFC_RX_COMMAND_3;
        SFC_RX_COMMAND_3: if (SFC_rx_this_command_complete) SFC_state_next = SFC_RX_COMMAND_4;
        SFC_RX_COMMAND_4: if (SFC_rx_this_command_complete) SFC_state_next = SFC_RX_COMMAND_5;
        SFC_RX_COMMAND_5: if (SFC_rx_this_command_complete) SFC_state_next = SFC_COMPLETE;
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
    if (SFC_state == SFC_RX_FLASH_METHOD) begin
        SFC_byte_sr <= { 5'b0, 1'b1 };
    end else if (rx_valid_r) begin
        SFC_byte_sr <= { SFC_byte_sr[4:0], SFC_byte_sr[5] };
    end
end

reg [15:0] SFC_address;
reg [7:0] SFC_data;
SFC_state_t SFC_store_state;

always @(posedge clk) begin
    SFC_store_state <= SFC_COMPLETE;
    unique case (1'b1)
        SFC_byte_sr[2]: SFC_address[15:8] <= rx_data_r;
        SFC_byte_sr[3]: SFC_address[7:0] <= rx_data_r;
        SFC_byte_sr[5]: begin
            SFC_data <= rx_data_r;
            if (rx_valid_r) SFC_store_state <= SFC_state;
        end
        default: ;
    endcase
end
always @(posedge clk) begin
    unique case (SFC_store_state)
        SFC_RX_COMMAND_0: commands[0] <= '{ SFC_address, SFC_data };
        SFC_RX_COMMAND_1: commands[1] <= '{ SFC_address, SFC_data };
        SFC_RX_COMMAND_2: commands[2] <= '{ SFC_address, SFC_data };
        SFC_RX_COMMAND_3: commands[3] <= '{ SFC_address, SFC_data };
        SFC_RX_COMMAND_4: commands[4] <= '{ SFC_address, SFC_data };
        SFC_RX_COMMAND_5: commands[5] <= '{ SFC_address, SFC_data };
        default: ;
    endcase
end
///// </SET_FLASH_CMD> /////
///// <FLASH_PROGRAM> /////

// We're occupying a BRAM slot, we might as well fill it (well, close enough) :)
// 1-indexed rather than 0-indexed so we can index by 'remaining'
reg [7:0] FP_program [1:2048];

///// <FLASH_PROGRAM:RX> /////
reg [12:0] FP_RX_remaining;
wire FP_RX_complete = (FP_RX_remaining == 0);

always @(posedge clk) begin
    if (!FLASH_PROGRAM_enable) begin
        FP_RX_remaining <= var_transfer_size;
    end else if (rx_valid_r) begin
        FP_RX_remaining <= FP_RX_remaining - 13'd1;
        FP_program[FP_RX_remaining] <= rx_data_r;
    end
end
///// </FLASH_PROGRAM:RX> /////

///// <FLASH_PROGRAM:EXEC> /////
reg [12:0] FP_EXEC_remaining;
reg [11:0] FP_EXEC_queue;
wire FP_EXEC_complete = (FP_EXEC_remaining == 0);

typedef enum {
    FP_EXEC_IDLE,
    FP_EXEC_FETCH,
    FP_EXEC,
    FP_EXEC_SUBMITTED
} FP_EXEC_state_t;
FP_EXEC_state_t FP_EXEC_state;

reg [5:0] FP_EXEC_cmd_sr;
always @(posedge clk) begin
    unique case (FP_EXEC_state)
        FP_EXEC_FETCH: FP_EXEC_cmd_sr <= 6'b000001;
        FP_EXEC: if (cart_ready) FP_EXEC_cmd_sr <= { FP_EXEC_cmd_sr[4:0], FP_EXEC_cmd_sr[5] };
        default : ;
    endcase
end

always @(posedge clk) begin
    if (!FLASH_PROGRAM_enable) begin
        FP_EXEC_queue <= 13'd0;
        FP_EXEC_remaining <= var_transfer_size;
    end else if (rx_valid_r) begin
        FP_EXEC_queue <= FP_EXEC_queue + 11'd1;
    end else if (FP_EXEC_state == FP_EXEC_SUBMITTED) begin
        FB_EXEC_queue <= FB_EXEC_queue - 11'd1;
        FP_EXEC_remaining <= FP_EXEC_remaining - 11'd1;
    end
end

FP_EXEC_state_t FP_EXEC_state_next;
always @(*) begin
    FP_EXEC_state_next = FP_EXEC_state;
    unique case (FP_EXEC_state)
        FP_EXEC_IDLE: if (FP_EXEC_queue > 0) FP_EXEC_state_next = FP_EXEC_FETCH_DATA;
        FP_EXEC_FETCH: FP_EXEC_state_next = FP_EXEC;
        FP_EXEC: if (cart_ready) FP_EXEC_state_next = FP_EXEC_SUBMITTED;
        FP_EXEC_SUBMITTED:  FP_EXEC_state_next = FP_EXEC_IDLE;
        default: ;
    endcase
end
always @(posedge clk) begin
    if (!FLASH_COMMAND_enable) FP_EXEC_state <= FP_EXEC_IDLE;
    else FP_EXEC_state <= FP_EXEC_state_next;
end

reg [15:0] FP_EXEC_payload_address;
reg [7:0] FP_EXEC_payload_data;
command_t FP_EXEC_command;

always @(posedge clk) begin
    if (!FLASH_PROGRAM_enable) begin
        FP_EXEC_payload_address <= var_address;
    end else if (FP_EXEC_state == FP_EXEC_FETCH) begin
        FP_EXEC_payload_data <= program[FP_EXEC_remaining];
        for (int i = 0; i < 6; i = i + 1) begin
            if (FB_EXEC_cmd_sr[i]) FP_EXEC_command <= command[i];
        end
    end else if (FP_EXEC_state == FP_EXEC_SUBMITTED) begin
        FP_EXEC_payload_address <= FP_EXEC_payload_address + 16'd1;
    end
end

reg FP_EXEC_have_payload;
always @(posedge clk) begin
    if (state != FP_EXEC) begin
        FP_EXEC_have_payload <= 1'b0;
    end
    if (cart_ready && !have_payload) begin
        // TODO
    end
end

///// </FLASH_PROGRAM:EXEC> /////

// Our cart FIFO is 512-deep, so... we're can't go further :D
reg [9:0] FP_pending_cart_commands;
logic [2:0] FP_cmd_idx;
reg [15:0] FP_address;

typedef enum {
    FP_RX_AND_EXEC,
    FP_EXEC_ONLY,
    FP_WAIT_CART_COMPLETE,
    FP_COMPLETE
} FP_state_t;
FP_state_t FP_state;
always @(posedge clk) FLASH_PROGRAM_complete <= (FP_state == FP_COMPLETE);

///// </FLASH_PROGRAM> /////

endmodule