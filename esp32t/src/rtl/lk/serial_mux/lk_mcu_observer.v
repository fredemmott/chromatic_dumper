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
    input enabled,
    input rx_valid,
    input [7:0] rx_data,
    output peer_t peer_o
);

typedef enum {
    S_DEFAULT,
/* TODO
    // pass through up to 3 bytes (len, payload0, payload1] to the MCU over UART
    // Cancel and reset to S_DEFAULT if the header byte is seen again
    S_MCU_V1_RX,
    // read the address byte from a v2 packet (header, addr, len, payload[...], crc byte)
    S_MCU_V2_RX_ADDR,
    // read the length byte from a v2 packet
    S_MCU_V2_RX_LEN,
    // pass through a specific number of bytes to the UART, then revert to DEFAULT.
    // Used for V2 packets once the length has been counted
    S_MCU_RX_COUNTED,
*/
    S_MCU_WAIT_RX, // Wait for the MCU to process the input byte
    S_55_WAIT_AA,
    S_LK_SERIAL_ID,
    S_L_WAIT_K,
    S_LK
} state_t;

state_t state = S_DEFAULT;
state_t rx_state;

always @(*) begin
    unique case (state)
        S_LK_SERIAL_ID: peer_o = P_LK_SERIAL_ID;
        S_LK: peer_o = P_LK;
        default: peer_o = P_MCU;
    endcase
end

state_t next_state;
state_t next_rx_state;

always @(*) begin
    next_state = state;
    next_rx_state = rx_state;
    if (!enabled) begin
        next_state = S_DEFAULT;
    end else begin
        unique case (state)
            S_DEFAULT: begin
                if (rx_valid) begin
                    if (rx_data == 8'h55) begin
                        next_rx_state = S_55_WAIT_AA;
                        next_state = S_MCU_WAIT_RX;
                    end else if (rx_data == "L") begin
                        next_rx_state = S_L_WAIT_K;
                        next_state = S_MCU_WAIT_RX;
                    end
                end
            end
            S_MCU_WAIT_RX: begin
                if (!rx_valid) next_state = rx_state;
            end
            S_55_WAIT_AA: begin
                if (rx_valid) begin
                    if (rx_data == 8'hAA) begin
                        next_state = S_LK_SERIAL_ID;
                    end else if (rx_data == 8'h55) begin
                        // Allow 0x55 0x55 .... 0xAA
                        next_state = S_MCU_WAIT_RX;
                    end
                end
            end
            S_L_WAIT_K: begin
                if (rx_valid) begin
                    if (rx_data == "K") begin
                        next_state = S_LK;
                    end else if (rx_data == "L") begin
                        // Allow LLLLLL...K
                        next_state = S_MCU_WAIT_RX;
                    end
                end
            end
            S_LK_SERIAL_ID, S_LK: /* terminal until reset */ ;
            default: next_state = S_DEFAULT;
        endcase
    end
end

always @(posedge clk) begin
    state <= next_state;
    rx_state <= next_rx_state;
end

endmodule