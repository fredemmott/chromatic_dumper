import lk_types::*;

module lk_cmd_set_pin_t(
    input  wire        clk,
    input  wire        reset,
    input  wire        enable,
    output reg         complete,
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    output reg         pin_audio
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

reg pin_audio_next;
always @(*) begin
    pin_audio_next = pin_audio;
    if (pin_bits[PIN_BIT_AUDIO]) begin
        pin_audio_next = value;
    end
end
always @(posedge clk) begin
    if (reset) begin
        pin_audio <= 1'b0;
    end else begin
        pin_audio <= pin_audio_next;
    end
end

always @(posedge clk) begin
    if (!enable) begin
        idx <= 3'd0;
    end else if (rx_valid_r) begin
        idx <= idx + 1'd1;
    end
end
always @(posedge clk) complete <= (idx >= 3'd4);

always @(posedge clk) begin
    if (!enable) begin
        pin_bits <= 32'd0;
    end else if (rx_valid_r) begin
        // byte [0..3]  bits
        //      [4]     value (1 or 0)
        unique case (idx)
            0: pin_bits[31:24] <= rx_data_r;
            1: pin_bits[23:16] <= rx_data_r;
            2: pin_bits[15:8] <= rx_data_r;
            3: pin_bits[7:0] <= rx_data_r;
            4: value <= rx_data_r[0];
            default: ;
        endcase
    end
end

endmodule