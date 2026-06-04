package lk_serial_mux;

typedef enum {
    P_INVALID, // error handling only
    P_MCU,
    P_LK_SERIAL_ID,
    P_LK
} peer_t;

endpackage

import lk_serial_mux::*;

module lk_mcu_observer_t(
    input clk,
    input reset,
    input enabled,
    input rx_valid,
    input [7:0] rx_data,
    output peer_t peer_o
);

// The UART can take multiple cycles to consume a single rx byte; to observe it, we want to detect
// rising edges
reg rx_valid_d;
always @(posedge clk) begin
    if (reset) begin
        rx_valid_d <= 1'b0;
    end else begin
        rx_valid_d <= rx_valid;
    end
end
wire rx_new_byte = rx_valid && !rx_valid_d;

reg [7:0] rx_data_d;
always @(posedge clk) begin
    if (reset) begin
        rx_data_d <= 8'd0;
    end else if (rx_new_byte) begin
        rx_data_d <= rx_data;
    end
end

// MCU V1: (header, addr, payload0, payload1)
//
// If we see the v1 header, we ignore the next 3 bytes
//
// MCU V2: (header, addr, len, payload[0..(len - 1)],crc)
//
// len is at most 14
// so, once we've seen the len, we have *at most* 15 bytes to ignore: 14 bytes of payload, then the CRC
// 15 fits in 4 bits, so:
reg [3:0] ignore_count;

typedef enum {
    S_DEFAULT,
    S_LK_SERIAL_ID,
    S_LK,
    // read the address byte from a v2 packet
    S_MCU_V2_RX_ADDR,
    // read the length byte from a v2 packet
    S_MCU_V2_RX_LEN,
    // Skip packets based on ignore_count
    S_MCU_RX_COUNTED
} state_t;

state_t state = S_DEFAULT;

always @(posedge clk) begin
    unique case (state)
        // Use the MCU V1 counter here, as:
        // - it only gets used if we go into S_MCU_RX_COUNTED
        // - for V2, we go into S_MCU_V2_RX_LEN first
        S_DEFAULT: ignore_count <= 4'd3;
        // Don't bother with rx_new_byte: it will be set on the last cycle we spend here.
        // + 1 for CRC
        S_MCU_V2_RX_LEN: ignore_count <= rx_data + 1;
        S_MCU_RX_COUNTED: if (rx_new_byte) ignore_count <= ignore_count - 4'd1;
        default: ;
    endcase
end

always @(*) begin
    unique case (state)
        S_LK_SERIAL_ID: peer_o = P_LK_SERIAL_ID;
        S_LK: peer_o = P_LK;
        default: peer_o = P_MCU;
    endcase
end

state_t next_state;

always @(*) begin
    next_state = state;
    if (!enabled) begin
        next_state = S_DEFAULT;
    end else begin
        unique case (state)
            S_DEFAULT: begin
                if (rx_new_byte) begin
                    if ((rx_data_d == 8'h55) && (rx_data == 8'hAA)) next_state = S_LK_SERIAL_ID;
                    else if ((rx_data_d == "L") && (rx_data == "K")) next_state = S_LK;
                    else if (rx_data == 8'h8A) next_state = S_MCU_RX_COUNTED; // MCU V1
                    else if (rx_data == 8'h8F) next_state = S_MCU_V2_RX_ADDR;
                end
            end
            S_MCU_V2_RX_ADDR: if (rx_new_byte) next_state = S_MCU_V2_RX_LEN;
            S_MCU_V2_RX_LEN: if (rx_new_byte) next_state = S_MCU_RX_COUNTED;
            S_MCU_RX_COUNTED: if (rx_new_byte && (ignore_count == 4'd1)) next_state = S_DEFAULT;
            S_LK_SERIAL_ID, S_LK: /* terminal until reset */ ;
            default: next_state = S_DEFAULT;
        endcase
    end
end

always @(posedge clk) state <= next_state;

endmodule