package cartio_serial_mux;

typedef enum {
    P_INVALID, // error handling only
    P_MCU,
    P_CARTIO_SERIAL_ID,
    P_CARTIO
} peer_t;

endpackage

import cartio_serial_mux::*;

module cartio_mcu_observer_t(
    input clk,
    input reset,
    input enabled,
    input rx_ready,
    input rx_valid,
    input [7:0] rx_data,
    output peer_t peer_o
);

// GAO shows that we get rx_valid pulses without rx_ready, which means that they're not 'popped' this leads
// to host writes of 'ABC' looking like 'AABBCC' etc.
//
// rx_ready is tied to the uart busy signal, and we get it a cycle late, so work on the negedge
logic rx_ready_d;
always @(posedge clk) begin
    rx_ready_d <= rx_ready;
end
wire rx_ready_negedge = {rx_ready_d, rx_ready} == 2'b10;

wire rx_new_byte = rx_valid && rx_ready_negedge;

localparam ACTIVATE = { "CartIO" };

reg [$bits(ACTIVATE) - 1:0] rx_data_sr;
always @(posedge clk) begin
    if (reset) begin
        rx_data_sr <= '{default:0};
    end else if (rx_new_byte) begin
        rx_data_sr <= { rx_data_sr[$bits(ACTIVATE) - 9:0] , rx_data };
    end
end
wire [7:0] rx_data_d = rx_data_sr[7:0];
wire [$bits(ACTIVATE) + 8 - 1:0] rx_data_view = { rx_data_sr, rx_data };

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

typedef enum logic [2:0] {
    S_DEFAULT,
    S_CARTIO_SERIAL_ID,
    S_CARTIO,
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
        S_MCU_V2_RX_LEN: ignore_count <= rx_data[3:0] + 1;
        S_MCU_RX_COUNTED: if (rx_new_byte) ignore_count <= ignore_count - 4'd1;
        default: ;
    endcase
end

always @(*) begin
    unique case (state)
        S_CARTIO_SERIAL_ID: peer_o = P_CARTIO_SERIAL_ID;
        S_CARTIO: peer_o = P_CARTIO;
        default: peer_o = P_MCU;
    endcase
end

state_t next_state;

always @(*) begin
    next_state = state;
    if (reset || !enabled) begin
        next_state = S_DEFAULT;
    end else begin
        unique case (state)
            S_DEFAULT: begin
                if (rx_new_byte) begin
                    if ((rx_data_view[23:0] == 24'hAA5590)) next_state = S_CARTIO_SERIAL_ID;
                    // "CartIO"
                    else if (rx_data_view == { ACTIVATE,  8'h00 }) next_state = S_CARTIO;
                    else if (rx_data == 8'h8A) next_state = S_MCU_RX_COUNTED; // MCU V1
                    else if (rx_data == 8'h8F) next_state = S_MCU_V2_RX_ADDR;
                end
            end
            S_MCU_V2_RX_ADDR: if (rx_new_byte) next_state = S_MCU_V2_RX_LEN;
            S_MCU_V2_RX_LEN: if (rx_new_byte) next_state = S_MCU_RX_COUNTED;
            S_MCU_RX_COUNTED: if (rx_new_byte && (ignore_count == 4'd1)) next_state = S_DEFAULT;
            S_CARTIO_SERIAL_ID, S_CARTIO: /* terminal until reset */ ;
            default: next_state = S_DEFAULT;
        endcase
    end
end

always @(posedge clk) state <= next_state;

endmodule