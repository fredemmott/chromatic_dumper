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


reg [7:0] rx_data_d;

always @(posedge usbClk) begin
    if (rx_valid) begin
        rx_data_d <= rx_data;
    end
end

typedef enum {
  S_IDLE,
  S_WAIT_ARG
} state_t;

state_t state;

logic have_command;
command_t command;
logic [7:0] arg;

assign have_command = (state == S_WAIT_ARG) && rx_valid;
assign command = have_command ? command_t'(rx_data_d) : CMD_NOP;
assign arg = rx_data;

always @(posedge usbClk) begin
    state <= state;

    if (usbReset) begin
        state <= S_IDLE;
    end else if (rx_valid) begin
        unique case (state)
            S_IDLE: state <= S_WAIT_ARG;
            S_WAIT_ARG: state <= S_IDLE;
            default: state <= S_IDLE;
        endcase
    end
end

always @(posedge cartClk) begin
    tx_valid <= 1'b0;
    tx_data <= 8'd0;

    if (!usbReset) begin
        unique case (command)
            CMD_PING: begin
                tx_valid <= 1'b1;
                tx_data <= ~arg;
            end
            CMD_GET_DATA: begin
                tx_valid <= 1'b1;
                tx_data <= cart_d_in;
            end
            default: /* nop */ ;
        endcase
    end
end

`define SET_PIN(TARGET, IDX) \
        if (arg[IDX + 4]) TARGET <= arg[IDX];
`define SET_TRISTATE_PIN(TARGET, IDX) \
        if (arg[IDX + 4]) begin \
            TARGET.oe <= 1'b1; \
            TARGET.value <= arg[IDX]; \
        end

always @(posedge cartClk) begin
    if (cartReset) begin
        cart_clk <= 1'b1;
        cart_wr <= 1'b1;
        cart_rd <= 1'b1;
        cart_cs <= 1'b1;
        cart_rst <= '{default: 0};
        cart_audio <= '{default: 0};

        cart_a <= 16'd0;
        cart_d_out <= 8'd0;
        cart_data_dir_e <= 1'b1; // read
    end else begin
        unique case (command)
            CMD_SET_OUTPUT_ENABLE: begin
                if (arg[OE_AUDIO + 4]) begin
                    cart_audio.oe <= arg[OE_AUDIO];
                end
                if (arg[OE_DATA + 4]) begin
                    cart_data_dir_e <= ~arg[OE_DATA];
                end
            end
            CMD_SET_PINS_A: begin
                `SET_PIN(cart_clk, SET_PINS_A_CLK);
                `SET_PIN(cart_wr, SET_PINS_A_WR);
                `SET_PIN(cart_rd, SET_PINS_A_RD)
                `SET_PIN(cart_cs, SET_PINS_A_CS)
            end
            CMD_SET_PINS_B: begin
                `SET_PIN(cart_a[15], SET_PINS_B_A15);
                `SET_TRISTATE_PIN(cart_rst, SET_PINS_B_RST);
                `SET_TRISTATE_PIN(cart_audio, SET_PINS_B_AUDIO);
            end
            CMD_SET_ADDRESS_MSB: begin
                cart_a[15:8] <= arg;
            end
            CMD_SET_ADDRESS_LSB: begin
                cart_a[7:0] <= arg;
            end
            CMD_SET_DATA: begin
                cart_d_out[7:0] <= arg;
            end
            default: ;
        endcase
    end
end

endmodule