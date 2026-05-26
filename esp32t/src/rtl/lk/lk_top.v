// Update the `set_clock_groups -asynchronous...` in evt1_x2.sdc if this changes
`define LK_CLOCK xClk

import lk_types::*;

module lk_top(
    input  wire        clk,
    input  wire        reset,

    // USB-CDC
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    output reg         tx_valid,
    output reg  [7:0]  tx_data,

    output reg         cart_enabled,

    output reg  [15:0] cart_a,
    output reg         cart_clk,
    output reg         cart_cs,
    output reg         cart_rd,
    output reg         cart_wr,
    output reg         cart_rst,
    output reg         cart_data_dir_e,   // 1 = read, 0 = write
    output reg  [7:0]  cart_d_out,        // data to write
    input  wire [7:0]  cart_d_in,         // data read from cart
    output reg         cart_audio
);

reg reset_r;
always @(posedge clk) reset_r <= reset;

// TODO: test removing this
wire cart_enabled_i;
always @(posedge clk) cart_enabled <= cart_enabled_i;

reg rx_valid_o;
reg [7:0] rx_data_o;
wire tx_valid_i;
wire [7:0] tx_data_i;
always @(posedge clk) begin
    rx_valid_o <= rx_valid;
    rx_data_o <= rx_data;
    tx_valid <= tx_valid_i;
    tx_data <= tx_data_i;
end

reg          cart_req_almost_full_o;
wire         cart_req_valid_i;
cart_req_t   cart_req_i;
cart_vars_t  cart_vars_i;
reg          cart_complete_o;
reg [7:0]    cart_complete_data_o;


lk_core u_core(
    .clk(clk),
    .reset(reset_r),

    .rx_valid(rx_valid_o),
    .rx_data(rx_data_o),
    .tx_valid(tx_valid_i),
    .tx_data(tx_data_i),

    .cart_enabled(cart_enabled_i),

    .cart_req_almost_full(cart_req_almost_full_o),
    .cart_req_valid(cart_req_valid_i),
    .cart_req(cart_req_i),
    .cart_vars(cart_vars_i),

    .cart_complete(cart_complete_o),
    .cart_complete_data(cart_complete_data_o)
);

cart_vars_t cart_vars;
always @(posedge clk) cart_vars <= cart_vars_i;

wire cart_complete;

logic cart_complete_sr [0:1];
logic [7:0] cart_complete_data_sr [0:1];
always @(posedge clk) begin
    cart_complete_sr[1] <= cart_complete_sr[0];
    cart_complete_data_sr[1] <= cart_complete_data_sr[0];
    cart_complete_sr[0] <= cart_complete;
    cart_complete_data_sr[0] <= cart_d_in;
end
assign cart_complete_o = cart_complete_sr[1];
assign cart_complete_data_o = cart_complete_data_sr[1];

reg [3:0] cart_req_count;
reg [3:0] cart_req_count_next;
always @(*) begin
    if (reset) begin
        cart_req_count_next = 0;
    end else begin
        unique case ({cart_complete_o, cart_req_valid_i})
            2'b10: cart_req_count_next = cart_req_count - 1'b1;
            2'b01: cart_req_count_next = cart_req_count + 1'b1;
            default: cart_req_count_next = cart_req_count;
        endcase
    end
end
always @(posedge clk) begin
    cart_req_count <= cart_req_count_next;
    cart_req_almost_full_o <= (cart_req_count_next >= 6);
end

// FIFO
reg req_dequeue;
wire reqs_empty;
cart_req_t req_q;
lk_cart_req_fifo_t u_cart_req_fifo(
    .clk(clk),
    .reset(reset_r),
    .enqueue(cart_req_valid_i),
    .dequeue(req_dequeue),
    .empty(reqs_empty),
    .in(cart_req_i),
    .out(req_q)
);

/// END FIFO
// Delay for routing
reg cart_req_valid_d;
reg cart_req_valid;
cart_req_t cart_req_d;
cart_req_t cart_req;

wire cart_req_started;
reg req_dequeue_d;
always @(posedge clk) begin
    if (reset_r) begin
        cart_req_valid <= 1'b0;
        cart_req_valid_d <= 1'b0;
        cart_req <= '{default: 0};
        cart_req_d <= '{default: 0};

        req_dequeue_d <= 1'b0;
        req_dequeue <= 1'b0;
    end else begin
        cart_req_valid_d <= !reqs_empty;
        cart_req_d <= req_q;
        cart_req_valid <= cart_req_valid_d;
        cart_req <= cart_req_d;

        req_dequeue_d <= cart_req_started;
        req_dequeue <= req_dequeue_d;
    end
end


lk_cart_t u_cart_executor(
    .clk(clk),
    .reset(reset_r),

    .req_valid(cart_req_valid),
    .req(cart_req),
    .req_started(cart_req_started),
    .req_complete(cart_complete),

    .hold_pin_audio(cart_vars.hold_pin_audio),

    .var_flash_we_pin(cart_vars.flash_we_pin),
    .var_dmg_read_cs_pulse(cart_vars.dmg_read_cs_pulse),
    .var_dmg_write_cs_pulse(cart_vars.dmg_write_cs_pulse),

    .cart_a(cart_a),
    .cart_clk(cart_clk),
    .cart_cs(cart_cs),
    .cart_rd(cart_rd),
    .cart_wr(cart_wr),
    .cart_rst(cart_rst),
    .cart_data_dir_e(cart_data_dir_e),
    .cart_d_out(cart_d_out),
    .cart_audio(cart_audio)
);

endmodule