module lk_cmd_dmg_cart_write_t (
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
    S_WAIT,
    S_COMPLETE
} state_t;
state_t state;
always @(posedge clk) complete <= (state == S_COMPLETE);

// byte [0..3]: address top 16 are always 0 for DMG
//      [4]   : value
reg [2:0] idx;
always @(posedge clk) begin
    cart_req_valid <= 1'b0;
    if (!enable) begin
        cart_req_address <= '0;
        cart_req_data <= '0;
    end else if (rx_valid) begin
        unique case (idx)
            2: cart_req_address[15:8] <= rx_data;
            3: cart_req_address[7:0] <= rx_data;
            4: begin
                cart_req_data <= rx_data;
                cart_req_valid <= 1'b1;
            end
            default: ;
        endcase
    end
end

state_t state_next;
always @(*) begin
    state_next = state;
    unique case(state)
        S_RX: if (rx_valid && (idx == 4)) state_next = S_WAIT;
        S_WAIT: if (cart_complete) state_next = S_COMPLETE;
        default: ;
    endcase
end

always @(posedge clk) begin
    if (!enable) begin
        state <= S_RX;
        idx <= '{default: 0};
    end else begin
        state <= state_next;
        if (rx_valid) idx <= idx + 1'd1;
    end
end

endmodule
