import lk_types::*;

module lk_cmd_set_pin_t(
    input  wire        clk,
    input  wire        reset,
    output reg         complete,
    input  cart_pins_t pins_in,
    output cart_pins_t pins_out
);

// A full 30 different pins are defined in LK_Device::SetPin, but this is the only one
// that's used by any callers
localparam PIN_BIT_AUDIO = 30;
// Command byte dealt with by caller
// 4 bytes: pin mask
// 1 byte: set_high
localparam RX_BYTE_COUNT = 5;

reg [31:0] pin_bits = '{default:0};
reg        pin_value = 0;

always @(*) begin
    pins_out = pins_in;
    if (pin_bits[PIN_BIT_AUDIO]) pins_out.audio = pin_value;
end

reg [2:0] idx = 0;

always @(posedge clk) begin
    if (reset) begin
        pin_bits <= '{default: 0};
        pin_value <= 0;
        idx <= 0;
    end else if (rx_valid)
        // byte [0..4]  bits
        //      [1]     value (1 or 0)
        idx <= idx + 1;
        // if (idx < 4)
        if (!idx[2]) pin_bits <= {pin_bits[23:0], rx_data};
        else begin
            pin_value <= rx_data;
        end
    end
end

endmodule