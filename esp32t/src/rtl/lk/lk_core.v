// cart_reader.v
// Chromatic FPGA Cart Reader
//
// Implements the LK (FlashGBX/Joey Jr) protocol over USB CDC serial (parallel byte
// interface from usbuvcuart_top EP3).
//
// Protocol summary (from LK_Device.py / hw_JoeyJr.py):
//   0x55 0xAA → device sends ID string (must contain "FW L"; must NOT contain "Joey")
//   'L' 'K'   → device sends 0xFF (LK firmware enabled)
//   'K' 'L'   → device sends 0xFF (LK firmware disabled)
//   0xA1 (QUERY_FW_INFO) → 1-byte size + 8 bytes info + optional name/flags
//   0xA3 (SET_MODE_DMG)   → ACK 0x01
//   0xA4/0xA5 (SET_VOLTAGE) → ACK 0x01
//   0xA6 (SET_VARIABLE)   → size(1)+key(4)+value(4), ACK 0x01
//   0xAB/0xAC (PULL_UPS)  → ACK 0x01
//   0xAD (GET_VARIABLE)   → size(1)+key(4), respond 4 bytes
//   0xA8 (SET_ADDR_INPUTS)→ ACK 0x01
//   0xA9 (CLK_TOGGLE)     → count(4), ACK 0x01
//   0xAE (GET_VAR_STATE)  → dump of all vars
//   0xAF (SET_VAR_STATE)  → receive all vars
//   0xB1 (DMG_CART_READ)  → read TRANSFER_SIZE bytes at ADDRESS from cart (or cache)
//   0xB2 (DMG_CART_WRITE) → addr(4)+val(1), ACK 0x01, cache invalidated
//   0xB3 (DMG_CART_WRITE_SRAM) → then recv TRANSFER_SIZE bytes, write each, ACK 0x01
//   0xB4 (DMG_MBC_RESET)  → ACK 0x01
//   0xB8 (DMG_SET_BANK_CHANGE_CMD) → recv params, ACK 0x01
//   0xBA (DMG_CART_READ_MEASURE) → same as 0xB1
//   0xD1 (DMG_FLASH_WRITE_BYTE) → addr(4)+val(1), ACK 0x01, cache invalidated
//   0xD3 (FLASH_PROGRAM)  → recv TRANSFER_SIZE bytes, write each, ACK 0x01 or 0x03
//   0xD4 (CART_WRITE_FLASH_CMD) → fc(1)+num(1)+[addr(4)+val(2)]×num, ACK 0x01
//   0xD5 (CALC_CRC32)     → ACK 0x01 (stub)
//   0xF2/0xF3 (PWR_ON/OFF)→ ACK 0x01
//   0xF4 (QUERY_CART_PWR) → respond 0x01 (cart always on)
//   0xF5 (SET_PIN)        → recv 5 bytes, ACK 0x01
//
//
// pClk is ~60 MHz (USB PHY clock from Gowin_PLL_UVC).
// Cart timing: 16-cycle CS/RD assertion (~267 ns) + 16-cycle wait → safe for 5V GB.

`default_nettype none

import lk_types::*;

module lk_core #(
    // Number of clock cycles CS/RD is asserted before latching data.
    // At 60 MHz, 16 cycles ≈ 267 ns (GB min CS low = 200 ns).
    parameter CART_RD_HOLD = 16,
    // Write pulse width (WR low).  At 60 MHz, 10 cycles ≈ 167 ns.
    parameter CART_WR_HOLD = 10,
    // Address-to-CS setup cycles.
    parameter CART_SETUP   = 4
)(
    input  wire        clk,
    input  wire        reset,

    // Parallel byte interface (EP3 via usbuvcuart_top)
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    output reg         tx_valid,
    output reg  [7:0]  tx_data,

    output reg         cart_enabled,

    input  wire        cart_req_almost_full,
    output reg         cart_req_valid,
    output cart_req_t  cart_req,
    output cart_vars_t cart_vars,

    input wire         cart_complete,
    input wire [7:0]   cart_complete_data
);
wire cart_req_ready = !cart_req_almost_full;
// This is just to simplify routing; this is safe because in top_v, reset is ~lk_enabled, synchronized with clk
reg reset_r = 1;
always @(posedge clk) reset_r <= reset;

vars_t vars = '{default: 0};

command_t command = CMD_INVALID;

wire QUERY_FW_INFO_complete;
wire QUERY_FW_INFO_tx_valid;
wire [7:0] QUERY_FW_INFO_tx_data;
lk_cmd_query_fw_info_t u_QUERY_FW_INFO(
    clk,
    (command == CMD_QUERY_FW_INFO),
    QUERY_FW_INFO_complete,
    QUERY_FW_INFO_tx_valid,
    QUERY_FW_INFO_tx_data
);

wire SET_VARIABLE_complete;
vars_t SET_VARIABLE_vars_out;
lk_cmd_set_variable_t u_SET_VARIABLE(
    clk,
    (command == CMD_SET_VARIABLE),
    SET_VARIABLE_complete,
    rx_valid,
    rx_data,
    vars,
    SET_VARIABLE_vars_out
);

wire GET_VARIABLE_complete;
wire GET_VARIABLE_tx_valid;
wire [7:0] GET_VARIABLE_tx_data;
lk_cmd_get_variable_t u_GET_VARIABLE(
    clk,
    (command == CMD_GET_VARIABLE),
    GET_VARIABLE_complete,
    rx_valid,
    rx_data,
    GET_VARIABLE_tx_valid,
    GET_VARIABLE_tx_data,
    vars
);

wire DMG_CART_READ_complete;
wire DMG_CART_READ_tx_valid;
wire [7:0] DMG_CART_READ_tx_data;
wire DMG_CART_READ_req_valid;
wire [15:0] DMG_CART_READ_req_address;

lk_cmd_dmg_cart_read_t u_DMG_CART_READ(
    clk,
    (command == CMD_DMG_CART_READ),
    DMG_CART_READ_complete,

    DMG_CART_READ_tx_valid,
    DMG_CART_READ_tx_data,

    cart_req_ready,
    DMG_CART_READ_req_valid,
    DMG_CART_READ_req_address,
    cart_complete,
    cart_complete_data,

    vars.address,
    vars.transfer_size
);

wire SET_PIN_complete;
reg hold_pin_audio;
lk_cmd_set_pin_t u_SET_PIN(
    clk,
    reset_r,
    (command == CMD_SET_PIN),
    SET_PIN_complete,
    rx_valid,
    rx_data,
    hold_pin_audio);

reg DMG_MBC_RESET_complete;
wire DMG_MBC_RESET_req_valid;
wire [15:0] DMG_MBC_RESET_req_address;
wire [7:0] DMG_MBC_RESET_req_data;
lk_cmd_dmg_mbc_reset_t u_DMG_MBC_RESET(
    clk,
    (command == CMD_DMG_MBC_RESET),
    DMG_MBC_RESET_complete,
    DMG_MBC_RESET_req_valid,
    DMG_MBC_RESET_req_address,
    DMG_MBC_RESET_req_data,
    cart_complete
);

always @(posedge clk) begin
    if (reset_r) begin
        cart_enabled <= 1'b0;
    end else if (command == CMD_SET_VOLTAGE_5V) begin
        cart_enabled <= 1'b1;
    end else if (command == CMD_SET_ADDR_AS_INPUTS) begin
        cart_enabled <= 1'b0;
    end
end

reg cart_req_valid_next;
cart_req_t cart_req_next;
always @(*) begin
    cart_req_valid_next = 1'b0;
    cart_req_next = '{default: 0};
    unique case (command)
        CMD_DMG_MBC_RESET: begin
            cart_req_valid_next = DMG_MBC_RESET_req_valid;
            cart_req_next = '{
                is_flash: 1'b0,
                is_write: 1'b1,
                address: DMG_MBC_RESET_req_address,
                data: DMG_MBC_RESET_req_data
            };
        end
        CMD_DMG_CART_READ: begin
            cart_req_valid_next = DMG_CART_READ_req_valid;
            cart_req_next = '{
                is_flash: 1'b0,
                is_write: 1'b0,
                address: DMG_CART_READ_req_address,
                data: 8'd0
            };
        end
        default: ;
    endcase
end
always @(posedge clk) begin
    cart_req_valid <= cart_req_valid_next;
    cart_req <= cart_req_next;
end

reg complete;
always @(*) begin
    complete = 1'b1;
    unique case (command)
        CMD_QUERY_FW_INFO: complete = QUERY_FW_INFO_complete;
        CMD_SET_VARIABLE: complete = SET_VARIABLE_complete;
        CMD_GET_VARIABLE: complete = GET_VARIABLE_complete;
        CMD_SET_PIN: complete = SET_PIN_complete;
        CMD_DMG_MBC_RESET: complete = DMG_MBC_RESET_complete;
        CMD_DMG_CART_READ: complete = DMG_CART_READ_complete;
        // Single-cycle and invalid
        default: ;
    endcase
end

reg exec_tx_valid;
reg [7:0] exec_tx_data;
always @(*) begin
    exec_tx_valid = 1'b0;
    exec_tx_data = 8'd0;

    unique case (command)
        CMD_DMG_MBC_RESET,
        CMD_SET_ADDR_AS_INPUTS,
        CMD_SET_PIN,
        CMD_SET_VARIABLE,
        CMD_SET_VOLTAGE_5V,
        CMD_STUB_NOOP_ACK: begin
            exec_tx_valid = complete;
            exec_tx_data = 8'd1;
        end
        CMD_QUERY_FW_INFO: begin
            exec_tx_valid = QUERY_FW_INFO_tx_valid;
            exec_tx_data = QUERY_FW_INFO_tx_data;
        end
        CMD_GET_VARIABLE: begin
            exec_tx_valid = GET_VARIABLE_tx_valid;
            exec_tx_data = GET_VARIABLE_tx_data;
        end
        CMD_DMG_CART_READ: begin
            exec_tx_valid = DMG_CART_READ_tx_valid;
            exec_tx_data = DMG_CART_READ_tx_data;
        end
        default: ;
    endcase
end

command_t cmd_rom [0:255];
integer i;
initial begin
    for (i = 0; i < 256; i++) begin
        cmd_rom[i] = CMD_INVALID;
    end

    // Must match `DEVICE_CMD` in `LK_Device.py`
    cmd_rom[8'hA1] = CMD_QUERY_FW_INFO;
    cmd_rom[8'hA3] = CMD_SET_MODE_DMG;
    cmd_rom[8'hA5] = CMD_SET_VOLTAGE_5V;
    cmd_rom[8'hA6] = CMD_SET_VARIABLE;
    cmd_rom[8'hA7] = CMD_SET_FLASH_CMD;
    cmd_rom[8'hA8] = CMD_SET_ADDR_AS_INPUTS;
    cmd_rom[8'hA9] = CMD_CLK_TOGGLE;
    cmd_rom[8'hAC] = CMD_DISABLE_PULLUPS;
    cmd_rom[8'hAD] = CMD_GET_VARIABLE;
    cmd_rom[8'hAE] = CMD_GET_VAR_STATE;
    cmd_rom[8'hAF] = CMD_SET_VAR_STATE;
    cmd_rom[8'hB1] = CMD_DMG_CART_READ;
    cmd_rom[8'hB2] = CMD_DMG_CART_WRITE;
    cmd_rom[8'hB3] = CMD_DMG_CART_WRITE_SRAM;
    cmd_rom[8'hB4] = CMD_DMG_MBC_RESET;
    cmd_rom[8'hB8] = CMD_DMG_SET_BANK_CHANGE_CMD;
    cmd_rom[8'hD1] = CMD_DMG_FLASH_WRITE_BYTE;
    cmd_rom[8'hD3] = CMD_FLASH_PROGRAM;
    cmd_rom[8'hD4] = CMD_CART_WRITE_FLASH_CMD;
    cmd_rom[8'hD5] = CMD_CALC_CRC32;
    cmd_rom[8'hF5] = CMD_SET_PIN;
end

enum {
    S_RESET,
    S_INIT,
    S_IDLE,
    S_DECODE,
    S_EXEC
} state = S_INIT;

command_t rx_command;
always @(posedge clk) rx_command <= cmd_rom[rx_data];

always @(posedge clk) begin
    if (reset_r) begin
        state <= S_RESET;
        command <= CMD_INVALID;
    end else begin
        unique case (state)
            S_RESET: state <= S_INIT;
            S_INIT: state <= S_IDLE;
            S_IDLE: if (rx_valid) state <= S_DECODE;
            S_DECODE: begin
                command <= rx_command;
                state <= S_EXEC;
            end
            S_EXEC: if (complete) begin
                state <= S_IDLE;
                command <= CMD_INVALID;
            end
            default: ;
        endcase
    end
end

reg next_tx_valid;
reg [7:0] next_tx_data;
always @(*) begin
    next_tx_valid = 1'b0;
    next_tx_data = 8'd0;
    unique case (state)
        S_RESET, S_IDLE, S_DECODE: ;
        S_INIT: begin
            next_tx_valid = 1'b1;
            next_tx_data = 8'hFF;
        end
        S_EXEC: begin
            next_tx_valid = exec_tx_valid;
            next_tx_data = exec_tx_data;
        end
        default: ;
    endcase
end

always @(posedge clk) begin
    tx_valid <= next_tx_valid;
    tx_data <= next_tx_data;
 end

vars_t vars_next;
always @(*) begin
    if (reset_r) begin
        vars_next = '{default: 0};
    end else begin
        vars_next = vars;
        unique case(command)
            CMD_SET_VARIABLE: vars_next = SET_VARIABLE_vars_out;
            CMD_DMG_CART_READ: vars_next.address = vars.address + vars.transfer_size;
            default: ;
        endcase
    end
end
always @(posedge clk) vars <= vars_next;

always @(posedge clk) begin
    cart_vars <= '{
        hold_pin_audio: hold_pin_audio,
        flash_we_pin: vars.flash_we_pin,
        dmg_read_cs_pulse: vars.dmg_read_cs_pulse,
        dmg_write_cs_pulse: vars.dmg_write_cs_pulse
    };
end

endmodule // cart_reader
`default_nettype wire
