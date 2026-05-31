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

// As usual, the packet is for 32-bit address and 16-bit data (AGB), but we only support
// DMG which is 16-bit addresses and 8-bit data
logic [7:0] commands_to_rx;
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

reg [2:0] idx;
wire rx_command_byte = (state == S_RX_COMMAND) && rx_valid_r;

always @(posedge clk) begin
    cart_req_valid <= 1'b0;
    if (rx_valid_r) begin
        unique case (idx)
            2: cart_req_address[15:8] <= rx_data_r;
            3: cart_req_address[7:0] <= rx_data_r;
            5: begin
                cart_req_valid <= 1'b1;
                cart_req_data <= rx_data_r;
            end
            default: ;
        endcase
    end
end

wire rx_command_count = (state == S_RX_HEADER) && rx_valid_r && idx[0];
always @(posedge clk) begin
    if (rx_command_count) begin
        commands_to_rx <= rx_data_r;
        commands_pending <= rx_data_r;
    end else if (rx_command_byte) begin
        if (idx == 0) commands_to_rx <= commands_to_rx - 1'd1;
    end

    if (cart_complete) commands_pending <= commands_pending - 1'd1;
end

wire rx_command_complete = rx_valid_r && (idx == 5);

// rx_command_count could also be called 'rx_header_complete'
wire reset_idx = rx_command_count || rx_command_complete || !enable;

always @(posedge clk) begin
    if (reset_idx) begin
        idx <= 1'd0;
    end else if (rx_valid_r) begin
        idx <= idx + 1'd1;
    end
end

wire rx_final_command = rx_command_complete && !commands_to_rx;

always @(posedge clk) begin
    if (!enable) begin
        state <= S_RX_HEADER;
    end else begin
        unique case(state)
            S_RX_HEADER: begin
                if (rx_command_count) begin
                    state <= (|rx_data_r) ? S_RX_COMMAND : S_COMPLETE;
                end
            end
            S_RX_COMMAND: if (rx_final_command) state <= S_WAIT;
            S_WAIT: if (!commands_pending) state <= S_COMPLETE;
            default: ;
        endcase
    end
end


endmodule