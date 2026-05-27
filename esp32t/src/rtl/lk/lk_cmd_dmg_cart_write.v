// byte [0..3]: address top 16 are always 0 for DMG
//      [4]   : value
module lk_cmd_dmg_cart_write_t #(
    parameter ADDRESS_MSB_RX_IDX = 2, // MSB that we actually pay attention to
    parameter DATA_RX_IDX = 4,
    parameter LAST_RX_IDX = 4
) (
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

localparam ADDRESS_LSB_RX_IDX = ADDRESS_MSB_RX_IDX + 1;
localparam RX_IDX_WIDTH = $clog2(LAST_RX_IDX + 1);

typedef enum {
    S_RX,
    S_EXEC,
    S_WAIT,
    S_COMPLETE
} state_t;
state_t state;
assign complete = (state == S_COMPLETE);
assign cart_req_valid = (state == S_EXEC);

reg [RX_IDX_WIDTH - 1:0] idx;

state_t state_next;
always @(*) begin
    state_next = state;
    unique case(state)
        S_RX: if (idx >= LAST_RX_IDX) state_next = S_EXEC;
        S_EXEC: state_next = S_WAIT;
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

reg [7:0] rx_buf [0:LAST_RX_IDX];
assign cart_req_address = { rx_buf[ADDRESS_MSB_RX_IDX], rx_buf[ADDRESS_LSB_RX_IDX] };
assign cart_req_data = rx_buf[DATA_RX_IDX];

integer i;
always @(posedge clk) begin
    if (rx_valid) begin
        for (i = 0; i < LAST_RX_IDX; i = i + 1) begin
            rx_buf[i] <= rx_buf[i + 1];
        end
        rx_buf[LAST_RX_IDX] <= rx_data;
    end
end

endmodule
