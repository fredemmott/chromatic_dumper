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

// Update the `set_clock_groups -asynchronous...` in evt1_x2.sdc if this changes
`define LK_CLOCK xClk

import lk_types::*;

module lk_top #(
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
    // Cartridge bus (top-level drives tristate from these)
    output reg  [15:0] cart_a,
    output reg         cart_clk,
    output reg         cart_cs,
    output reg         cart_rd,
    output reg         cart_wr,
    output reg         cart_rst,
    output reg         cart_data_dir_e,   // 1 = read, 0 = write
    output reg  [7:0]  cart_d_out,        // data to write
    input  wire [7:0]  cart_d_in,         // data read from cart
    output reg         cart_audio
);
// This is just to simplify routing; this is safe because in top_v, reset is ~lk_enabled, synchronized with clk
reg reset_r = 1;
always @(posedge clk) reset_r <= reset;

vars_t vars = '{default: 0};

command_t command = CMD_INVALID;

reg cart_complete_r;

wire QUERY_FW_INFO_complete;
wire QUERY_FW_INFO_tx_valid;
wire [7:0] QUERY_FW_INFO_tx_data;
lk_cmd_query_fw_info_t QUERY_FW_INFO(
    clk,
    (command == CMD_QUERY_FW_INFO),
    QUERY_FW_INFO_complete,
    QUERY_FW_INFO_tx_valid,
    QUERY_FW_INFO_tx_data
);

wire SET_VARIABLE_complete;
vars_t SET_VARIABLE_vars_out;
lk_cmd_set_variable_t SET_VARIABLE_info(
    clk,
    (command == CMD_SET_VARIABLE),
    SET_VARIABLE_complete,
    rx_valid,
    rx_data,
    vars,
    SET_VARIABLE_vars_out
);

wire SET_PIN_complete;
cart_pins_t cart_idle_pins;
cart_pins_t SET_PIN_pins;
lk_cmd_set_pin_t SET_PIN(
    clk,
    (command == CMD_SET_PIN),
    SET_PIN_complete,
    rx_valid,
    rx_data,
    cart_idle_pins,
    SET_PIN_pins);
always @(posedge clk) begin
    if (reset_r) begin
        cart_idle_pins <= '{
            address: 16'hFFFF,
            clk: 1'b1,
            cs: 1'b1,
            rd: 1'b1,
            wr: 1'b1,
            rst: 1'b1,
            data_dir_e: 1'b1, // is read
            data: 8'd0,
            audio: 1'b0
        };
    end else begin
        if (SET_PIN_complete) cart_idle_pins <= SET_PIN_pins;
    end
end

reg DMG_MBC_RESET_complete;
cart_req_t DMG_MBC_RESET_cart_req;
lk_cmd_dmg_mbc_reset_t cmd_dmg_mbc_reset(
    clk,
    (command == CMD_DMG_MBC_RESET),
    DMG_MBC_RESET_complete,
    DMG_MBC_RESET_cart_req,
    cart_complete_r
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

cart_req_t cart_req;
assign cart_req = (command == CMD_DMG_MBC_RESET) ? DMG_MBC_RESET_cart_req : '{default: 0};
cart_pins_t cart;
reg cart_complete;
lk_cart_t cart_executor(
    clk,
    reset_r,
    cart_req,
    cart_complete,
    cart_idle_pins,
    cart,
    vars.flash_we_pin,
    vars.dmg_read_cs_pulse,
    vars.dmg_write_cs_pulse
);
always @(posedge clk) cart_complete_r <= cart_complete;
assign cart_a = cart.address;
assign cart_clk = cart.clk;
assign cart_cs = cart.cs;
assign cart_rd = cart.rd;
assign cart_wr = cart.wr;
assign cart_rst = cart.rst;
assign cart_data_dir_e = cart.data_dir_e;
assign cart_d_out = cart.data;
assign cart_audio = cart.audio;

reg complete;
always @(*) begin
    complete = 1'b1;
    unique case (command)
        CMD_QUERY_FW_INFO: complete = QUERY_FW_INFO_complete;
        CMD_SET_VARIABLE: complete = SET_VARIABLE_complete;
        CMD_SET_PIN: complete = SET_PIN_complete;
        CMD_DMG_MBC_RESET: complete = DMG_MBC_RESET_complete;
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

always @(posedge clk) begin
    if (reset_r) vars <= '{default: 0};
    else if (SET_VARIABLE_complete) vars <= SET_VARIABLE_vars_out;
end

endmodule // cart_reader
`default_nettype wire
