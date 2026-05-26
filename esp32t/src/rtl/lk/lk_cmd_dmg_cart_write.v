module lk_cmd_dmg_cart_write_t(
    input wire clk,
    input wire enable,
    output wire complete,

    input wire rx_valid,
    input wire [7:0] rx_data,

    output reg cart_req_valid,
    output reg [15:0] cart_req_address,
    output reg [7:0] cart_req_data,
    input reg cart_complete
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

reg [2:0] idx = 0;
state_t next_state;

always @(*) begin
    next_state = state;
    unique case(state)
        S_RX: if (idx == 4) next_state = S_EXEC;
        S_EXEC: next_state = S_WAIT;
        S_WAIT: if (cart_complete) next_state = S_COMPLETE;
        default: ;
    endcase
end
always @(posedge clk) begin
    if (!enable) begin
        idx <= 0;
        state <= S_RX;
    end else begin
        if (rx_valid) idx <= idx + 1;
        state <= next_state;
    end
end

always @(posedge clk) begin
    // byte [0..3]: address top 16 are always 0 for DMG
    //      [4]   : value
    unique case (idx)
        2: cart_req_address[15:8] <= rx_data;
        3: cart_req_address[7:0] <= rx_data;
        4: cart_req_data <= rx_data;
        default: ;
    endcase
end

endmodule