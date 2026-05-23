import lk_types::*;

module lk_cmd_set_pin_t(
    input  wire        clk,
    input  wire        enable,
    output reg         complete,
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    input  cart_pins_t pins_in,
    output cart_pins_t pins_out
);

// A full 30 different pins are defined in LK_Device::SetPin, but this is the only one
// that's used by any callers
localparam PIN_BIT_AUDIO = 30;

reg rx_valid_r;
reg [7:0] rx_data_r;
always @(posedge clk) begin
    rx_valid_r <= rx_valid;
    rx_data_r <= rx_data;
end

reg [31:0] pin_bits;
reg [2:0] idx;
reg value;

always @(*) begin
    pins_out = pins_in;
    if (pin_bits[PIN_BIT_AUDIO]) pins_out.audio <= value;
end

always @(posedge clk) begin
    complete <= 0;
    if (!enable) begin
        idx <= 3'd0;
    end else if (rx_valid_r) begin
        idx <= idx + 1'b1;
        // byte [0..3]  bits
        //      [4]     value (1 or 0)
        unique case (idx)
            3: pin_bits[7:0] <= rx_data_r;
            4: begin
                value <= rx_data[0];
                complete <= 1'b1;
            end
            default: ;
        endcase
    end
end

endmodule