// Send an ack when complete
module lk_cmd_set_pin_t(
    input wire clk,
    input wire en,
    output wire complete,
    input wire rx_valid,
    input wire [7:0] rx_data,
    output reg cart_audio
);
    // A full 30 different pins are defined in LK_Device::SetPin, but this is the only one
    // that's used by any callers
    localparam PIN_BIT_AUDIO = 30;
    // Command byte dealt with by caller
    // 4 bytes: pin mask
    // 1 byte: set_high
    localparam RX_BYTE_COUNT = 5;

    reg [31:0] pin_bits = '{default:0};

    reg [2:0] index;

    assign complete = (index == RX_BYTE_COUNT);

    always @(posedge clk) begin
        if (!en) begin
            index <= 0;
            cart_audio <= 0;
        end else if (rx_valid && !complete) begin
            index <= index + 1;
            // if (index < 4)
            if (!index[2]) pin_bits <= {pin_bits[23:0], rx_data};
            else if (pin_bits[PIN_BIT_AUDIO]) cart_audio <= rx_data[0];
        end
    end
endmodule