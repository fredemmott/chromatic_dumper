module lk_cmd_cart_write_flash_cmd_t(
    input  wire        clk,
    input  wire        enable,
    output reg         complete,

    input  reg         rx_valid,
    input  reg  [7:0]  rx_data,

    output reg         cart_req_valid,
    output reg  [15:0] cart_req_address,
    output reg  [7:0]  cart_req_data,
    input  reg         cart_complete
);

reg rx_valid_r;
reg [7:0] rx_data_r;
always @(posedge clk) begin
    rx_valid_r <= rx_valid;
    rx_data_r <= rx_data;
end

reg [2:0] idx;

// As usual, the packet is for 32-bit address and 16-bit data (AGB), but we only support
// DMG which is 16-bit addresses and 8-bit data
logic [7:0] commands_remaining;
logic [7:0] commands_pending;

typedef enum {
    S_RX_HEADER,
    S_RX_COMMAND,
    S_WAIT,
    S_COMPLETE
} state_t;

state_t state;
always @(posedge clk) complete <= (state == S_COMPLETE);

// byte 0: 'is flashcart' (unused)
//      1: number of commands
//      ...: commands

// command bytes [0..3]: address
//               [4..5]: data (command)

wire this_command_received = (idx >= 3'd5);

reg [2:0] next_idx;
always @(*) begin
    next_idx = idx;
    if (rx_valid_r) begin
        next_idx = idx + 1'd1;
        unique case (state)
            S_RX_HEADER: if (idx[0]) next_idx = 0; // 1 -> S_RX_COMMAND
            S_RX_COMMAND: if (this_command_received) next_idx = 0; // next command
            default: ;
        endcase
    end
end
always @(posedge clk) idx <= enable ? next_idx : 0;

wire all_commands_received = this_command_received && !commands_remaining;
wire all_commands_complete = (commands_pending == 0);

state_t next_state;
always @(*) begin
    next_state = state;
    unique case(state)
        S_RX_HEADER: if (idx[0]) next_state = S_RX_COMMAND;
        S_RX_COMMAND: if (all_commands_received) next_state = S_WAIT;
        S_WAIT: if (all_commands_complete) next_state = S_COMPLETE;
        // S_COMPLETE: /* terminal until !enable */ ;
        default: ;
    endcase
end
always @(posedge clk) state <= enable ? next_state : S_RX_HEADER;

always @(posedge clk) begin
    if (state == S_RX_HEADER) commands_pending <= rx_data_r;
    else if (cart_complete) commands_pending <= commands_pending - 1'd1;
end
always @(posedge clk) begin
    if (state == S_RX_HEADER) begin
        commands_remaining <= rx_data_r;
    end
    else if ((state == S_RX_COMMAND) && idx == 0) commands_remaining <= commands_remaining - 1'd1;
end

wire rx_command = rx_valid_r && (state == S_RX_COMMAND);

always @(posedge clk) begin
    cart_req_valid <= 1'b0;
    if (!enable) begin
        cart_req_address <= '0;
        cart_req_data <= '0;
    end else begin
        if (rx_command) begin
            unique case (idx)
                2: cart_req_address[15:8] <= rx_data_r;
                3: cart_req_address[7:0] <= rx_data_r;
                5: begin
                    cart_req_data <= rx_data_r;
                    cart_req_valid <= 1'b1;
                end
                default: ; // others are MSB data for AGB only
            endcase
        end
    end
end

endmodule