import lk_types::*;

module lk_cmd_cart_write_flash_cmd_t (
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

typedef enum {
    S_RX,
    S_EXEC,
    S_WAIT,
    S_COMPLETE
} state_t;
state_t state;
assign complete = (state == S_COMPLETE);
assign cart_req_valid = (state == S_EXEC);

reg [2:0] idx;

state_t state_next;
always @(*) begin
    state_next = state;
    unique case(state)
        S_RX: if (idx >= 6) state_next = S_EXEC;
        S_EXEC: state_next = S_WAIT;
        S_WAIT: if (cart_complete) state_next = S_COMPLETE;
        default: ;
    endcase
end

always @(posedge clk) begin
    if (!enable) begin
        state <= S_RX;
        idx <= 3'd0;
    end else begin
        state <= state_next;
        if (rx_valid) idx <= idx + 3'd1;
    end
end

// byte [0]:    is flash cart (unused)
//      [1..4]: address (2 MSB for AGB only)
//      [5..6]: data (MSB for AGB only)
// ... so we only need the last 4 rx bytes
reg [7:0] rx_buf [0:3];
assign cart_req_address = { rx_buf[0], rx_buf[1] };
assign cart_req_data = rx_buf[3];
integer i;
always @(posedge clk) begin
    if (rx_valid) begin
        for (i = 0; i < 3; i = i + 1) begin
            rx_buf[i] <= rx_buf[i + 1];
        end
        rx_buf[3] <= rx_data;
    end
end

endmodule