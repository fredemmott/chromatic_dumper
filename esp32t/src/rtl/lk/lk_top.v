import lk_types::*;

module lk_top(
    input  wire        usbClk,
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
    output reg         cart_data_dir_e,   // 1 = read, 0 = write
    output reg  [7:0]  cart_d_out,        // data to write
    input  wire [7:0]  cart_d_in,         // data read from cart
    output tristate_pin_t cart_rst,
    output tristate_pin_t cart_audio
);
assign cart_enabled = 1'b1;

(* syn_preserve *) reg [1:0] lk_cdc_usb_reset;
always @(posedge usbClk or posedge reset) begin
    if (reset) lk_cdc_usb_reset <= 2'b11;
    else       lk_cdc_usb_reset <= { lk_cdc_usb_reset[0], 1'b0 };
end
wire usbReset = lk_cdc_usb_reset[1];

(* syn_preserve *) reg [1:0] lk_cdc_cart_reset;
always @(posedge cartClk or posedge reset) begin
    if (reset) lk_cdc_cart_reset <= 2'b11;
    else       lk_cdc_cart_reset <= { lk_cdc_cart_reset[0], 1'b0 };
end
wire cartReset = lk_cdc_cart_reset[1];


reg [15:0] rx_buf;

always @(posedge usbClk) begin
    if (rx_valid) begin
        rx_buf <= { rx_buf[7:0], rx_data };
    end
end

typedef enum {
  S_IDLE,
  S_WAIT_ARG8A,
  S_WAIT_ARG8B
} state_t;

state_t state;

logic have_command;
command_t command;
logic [15:0] arg16;
logic [7:0] arg8a;
logic [7:0] arg8b;

assign have_command = (state == S_WAIT_ARG8B) && rx_valid;
assign command = have_command ? command_t'(rx_buf[15:8]) : CMD_IDLE;
assign arg8a = rx_buf[7:0];
assign arg8b = rx_data;
assign arg16 = {arg8a, arg8b};

always @(posedge usbClk) begin
    state <= state;

    if (usbReset) begin
        state <= S_IDLE;
    end else if (rx_valid) begin
        unique case (state)
            S_IDLE: state <= S_WAIT_ARG8A;
            S_WAIT_ARG8A: state <= S_WAIT_ARG8B;
            S_WAIT_ARG8B: state <= S_IDLE;
            default: state <= S_IDLE;
        endcase
    end
end

logic tx_valid_next;
logic [7:0] tx_data_next;

always @(posedge cartClk) begin
    tx_valid <= 1'b0;
    tx_data <= 8'd0;

    tx_valid_next <= 1'b0;
    tx_data_next <= 8'd0;

    if (!usbReset) begin
        if (have_command) begin
            tx_valid <= 1'b1;
            tx_data <= 8'(command);

            tx_valid_next <= 1'b1;
            unique case (command)
                CMD_PING: begin
                    tx_data_next <= ~arg8a;
                end
                CMD_GET_DATA: begin
                    tx_data_next <= cart_d_in;
                end
                default: begin
                    tx_data_next <= 8'h01;
                end
            endcase
        end else if (tx_valid_next ) begin
            tx_valid <= 1'b1;
            tx_data <= tx_data_next;
        end
    end
end

`define SET_PIN(TARGET, IDX) \
        if (arg8a[IDX]) TARGET <= arg8b[IDX];
`define SET_TRISTATE_PIN(TARGET, IDX) \
        if (arg8a[IDX]) begin \
            TARGET.oe <= 1'b1; \
            TARGET.value <= arg8b[IDX]; \
        end

        // A15 is handled in SET_ADDRESS
always @(posedge cartClk) begin
    if (cartReset) begin
        cart_clk <= 1'b1;
        cart_wr <= 1'b1;
        cart_rd <= 1'b1;
        cart_cs <= 1'b1;
        cart_rst <= '{default: 0};
        cart_audio <= '{default: 0};
    end else if (command == CMD_SET_PINS) begin
        `SET_PIN(cart_clk, SET_PIN_CLK);
        `SET_PIN(cart_wr, SET_PIN_WR);
        `SET_PIN(cart_rd, SET_PIN_RD)
        `SET_PIN(cart_cs, SET_PIN_CS)
        `SET_TRISTATE_PIN(cart_rst, SET_PIN_RST);
        `SET_TRISTATE_PIN(cart_audio, SET_PIN_AUDIO);
    end else if (command == CMD_SET_OUTPUT_ENABLE) begin
        if (arg8a[OE_AUDIO]) begin
            cart_audio.oe <= arg8b[OE_AUDIO];
        end
    end
end

always @(posedge cartClk) begin
    if (cartReset) begin
        cart_data_dir_e <= 1'b1; // read
    end else if ((command == CMD_SET_OUTPUT_ENABLE) && arg8a[OE_DATA]) begin
        cart_data_dir_e <= ~arg8b[OE_DATA];
    end
end

always @(posedge cartClk) begin
    if (cartReset) begin
        cart_a <= '{default: 0};
    end else if (command == CMD_SET_ADDRESS) begin
        cart_a <= arg16;
    end else if ((command == CMD_SET_PINS) && arg8b[SET_PIN_A15]) begin
        cart_a[15] <= arg8b[SET_PIN_A15];
    end
end

always @(posedge cartClk) begin
    if (cartReset) begin
        cart_d_out <= '{default: 0};
    end else if (command == CMD_SET_DATA) begin
        cart_d_out <= arg8a;
    end
end

endmodule