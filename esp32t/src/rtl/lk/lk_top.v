import lk_types::*;

module lk_top(
    input  wire        coreClk,
    input  wire        cartClk,
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

(* syn_preserve *) reg [1:0] lk_cdc_core_reset;
always @(posedge coreClk or posedge reset) begin
    if (reset) lk_cdc_core_reset <= 2'b11;
    else       lk_cdc_core_reset <= { lk_cdc_core_reset[0], 1'b0 };
end
wire coreReset = lk_cdc_core_reset[1];

(* syn_preserve *) reg [1:0] lk_cdc_cart_reset;
always @(posedge cartClk or posedge reset) begin
    if (reset) lk_cdc_cart_reset <= 2'b11;
    else       lk_cdc_cart_reset <= { lk_cdc_cart_reset[0], 1'b0 };
end
wire cartReset = lk_cdc_cart_reset[1];

logic cart_enabled_i;
(* syn_preserve *) reg [1:0] lk_cdc_cart_enabled;
always @(posedge cartClk) begin
    lk_cdc_cart_enabled[0] <= cart_enabled_i;
    lk_cdc_cart_enabled[1] <= lk_cdc_cart_enabled[0];
end
assign cart_enabled = lk_cdc_cart_enabled[1];

reg rx_valid_o;
reg [7:0] rx_data_o;
wire tx_valid_i;
wire [7:0] tx_data_i;
always @(posedge coreClk) begin
    rx_valid_o <= rx_valid;
    rx_data_o <= rx_data;
    tx_valid <= tx_valid_i;
    tx_data <= tx_data_i;
end

logic        cart_req_valid_i;
cart_req_t   cart_req_i;
cart_vars_t  cart_vars_i;
logic        cart_complete_o;
logic [7:0]  cart_complete_data_o;
wire cart_complete_dequeue;

(* syn_netlist_hierarchy = 1 *)
lk_core u_core(
    .clk(coreClk),
    .reset(coreReset),

    .rx_valid(rx_valid_o),
    .rx_data(rx_data_o),
    .tx_valid(tx_valid_i),
    .tx_data(tx_data_i),

    .cart_enabled(cart_enabled_i),

    .enqueue_o(cart_req_valid_i),
    .req_o(cart_req_i),
    .vars_o(cart_vars_i),

    .dequeue_o(cart_complete_dequeue),
    .cart_complete(cart_complete_o),
    .cart_complete_data(cart_complete_data_o)
);

wire cart_complete;

typedef struct packed {
    logic [7:0] cart_d_in;
    logic [23:0] _padding;
} fifo_response_t;
fifo_response_t cart_complete_data;
fifo_response_t cart_complete_q;
logic cart_complete_enqueue;

wire cart_complete_empty;
assign cart_complete_o = !(coreReset | cart_complete_empty);
assign cart_complete_data_o = cart_complete_q.cart_d_in;


logic cart_complete_enqueue_d;
fifo_response_t cart_complete_data_d;
always @(posedge cartClk) begin
    cart_complete_enqueue <= cart_complete;
    cart_complete_data <= '{
        cart_d_in: cart_d_in,
        _padding: '0
    };

    cart_complete_enqueue_d <= cart_complete_enqueue;
    cart_complete_data_d <= cart_complete_data;
end

lk_cart_fifo_t u_cart_complete_fifo(
    .WrClk(cartClk),
    .WrEn(cart_complete_enqueue_d),
    .Data(cart_complete_data_d),

    .RdClk(coreClk),
    .RdEn(cart_complete_dequeue),
    .Q(cart_complete_q),
    .Empty(cart_complete_empty),

    .Almost_Full(), // Reader is 4x faster than writer, don't bother with this wire
    .Full() // ditto
);

// FIFO
logic req_enqueue;
cart_req_t req_enqueue_data;
logic req_dequeue;
logic reqs_empty;

typedef struct packed {
    logic        is_flash;
    logic        is_write;
    logic        wait_for_status;
    logic [15:0] address;
    logic [7:0]  data;

    logic [1:0]  flash_we_pin;
    logic        dmg_read_cs_pulse;
    logic        dmg_write_cs_pulse;

    logic        _padding; // 32-bit
} fifo_req_t;
fifo_req_t req_data;
fifo_req_t req_q;

lk_cart_fifo_t u_cart_req_fifo(
    .WrClk(coreClk),
    .WrEn(req_enqueue),
    .Data(req_data),

    .RdClk(cartClk),
    .RdEn(req_dequeue),
    .Q(req_q),
    .Empty(reqs_empty),

    .Almost_Full(),
    .Full()
);

/// END FIFO
assign req_enqueue = cart_req_valid_i;
assign req_data = '{
    is_flash:        cart_req_i.is_flash,
    is_write:        cart_req_i.is_write,
    wait_for_status: cart_req_i.wait_for_status,
    address:         cart_req_i.address,
    data:            cart_req_i.data,

    flash_we_pin: cart_vars_i.flash_we_pin,
    dmg_read_cs_pulse: cart_vars_i.dmg_read_cs_pulse,
    dmg_write_cs_pulse: cart_vars_i.dmg_write_cs_pulse,

    _padding: '0
};

logic cart_req_valid;
logic cart_req_started;

always @(posedge cartClk) req_dequeue <= cart_req_started && !reqs_empty;

(* syn_preserve *) reg [1:0] lk_cdc_hold_pin_audio;
always @(posedge cartClk) begin
    lk_cdc_hold_pin_audio[0] <= cart_vars_i.hold_pin_audio;
    lk_cdc_hold_pin_audio[1] <= lk_cdc_hold_pin_audio[0];
end
assign hold_pin_audio = lk_cdc_hold_pin_audio[1];

fifo_req_t req_q_d;
always @(posedge cartClk) begin
    cart_req_valid <= !reqs_empty;
    req_q_d <= req_q;
end

(* syn_netlist_hierarchy = 1 *)
lk_cart_t u_cart_executor(
    .clk(cartClk),
    .reset(cartReset),

    .req_valid(cart_req_valid),
    .req('{
        is_flash: req_q_d.is_flash,
        is_write: req_q_d.is_write,
        wait_for_status: req_q_d.wait_for_status,
        address: req_q_d.address,
        data: req_q_d.data
    }),
    .req_started(cart_req_started),
    .req_complete(cart_complete),

    .hold_pin_audio(hold_pin_audio),

    .var_flash_we_pin(req_q_d.flash_we_pin),
    .var_dmg_read_cs_pulse(req_q_d.dmg_read_cs_pulse),
    .var_dmg_write_cs_pulse(req_q_d.dmg_write_cs_pulse),

    .cart_a(cart_a),
    .cart_clk(cart_clk),
    .cart_cs(cart_cs),
    .cart_rd(cart_rd),
    .cart_wr(cart_wr),
    .cart_rst(cart_rst),
    .cart_data_dir_e(cart_data_dir_e),
    .cart_d_in(cart_d_in),
    .cart_d_out(cart_d_out),
    .cart_audio(cart_audio)
);

endmodule