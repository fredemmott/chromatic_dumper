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

typedef enum {
    S_RX,
    S_UPDATE,
    S_COMPLETE
} state_t;
state_t state;
assign complete = (state == S_COMPLETE);

logic [31:0] pin_bits;
logic value;

reg [2:0] idx;

state_t next_state;
always @(*) begin
    next_state = state;
    unique case (state)
        S_RX: if (rx_valid && (idx == 3'd4)) next_state = S_UPDATE;
        S_UPDATE: next_state = S_COMPLETE;
        default: ;
    endcase
end
always @(posedge clk) begin
    if (!enable) state <= S_RX;
    else state <= next_state;
end

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
    if (!enable) idx <= 3'd0;
    else if (rx_valid) idx <= idx + 1'd1;
end

always @(posedge clk) begin
    if (rx_valid) begin
        // byte [0..3] : pin bits
        //      [4]    : value
        unique case (idx)
            0: pin_bits[31:24] <= rx_data;
            1: pin_bits[23:16] <= rx_data;
            2: pin_bits[15:8] <= rx_data;
            3: pin_bits[7:0] <= rx_data;
            4: value <= rx_data[0];
            default: ;
        endcase
    end
end

endmodule