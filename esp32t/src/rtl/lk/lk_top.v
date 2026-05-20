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
    input  wire        PHY_CLKOUT,

    // Parallel byte interface (EP3 via usbuvcuart_top)
    input  wire        RX_VALID,
    input  wire [7:0]  RX_DATA,
    output reg         TX_VALID,
    output reg  [7:0]  TX_DATA,

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

// USB data is on PHY_CLKOUT
// Cartridge pins and all logic in top.v is on xClk instead, or derived clocks like pClk
// Using xClk as this module's native clock as:
// - we're limited by the cartridge speed anyway
// - it greatly simplifies the MUX in top.v
// - it's much less work than trying to CDC the cartridge and the cartridge state machine :)
// The two clocks are close, but xClk is slightly faster, so we can cork on the fifo
//
// xClk         67.109Mhz   14.901ns
// PHY_CLKOUT   60mhz       16.667ns

reg        tx_valid = 0;
reg  [7:0] tx_data = '0;
reg tx_full = 0;

wire       TX_EMPTY;
wire       TX_RD_EN = ~TX_EMPTY;
wire [7:0] TX_Q;

// lk_top -> USB
lk_usb_fifo_t usb_tx_fifo(
    .WrClk(clk),
    .Full(tx_full),
    .WrEn(tx_valid),
    .Data(tx_data),

    .RdClk(PHY_CLKOUT),
    .Empty(TX_EMPTY),
    .RdEn(TX_RD_EN),
    .Q(TX_Q)
);
assign TX_VALID = ~TX_EMPTY;
assign TX_DATA = TX_Q;

reg        rx_valid;
reg  [7:0] rx_data;

wire       rx_empty;
wire       rx_rd_en = ~rx_empty;
wire [7:0] rx_q;
reg        RX_FULL = 0; // unused
// USB -> LK_TOP
lk_usb_fifo_t usb_rx_fifo(
    .WrClk(PHY_CLKOUT),
    .Full(RX_FULL),
    .WrEn(RX_VALID),
    .Data(RX_DATA),

    .RdClk(clk),
    .Empty(rx_empty),
    .RdEn(rx_rd_en),
    .Q(rx_q)
);
assign rx_valid = ~rx_empty;
assign rx_data = rx_q;

vars_t vars = '{default: 0};

command_t command_from_rx_data;
always @(*) begin
    command_from_rx_data = CMD_IDLE;
    case (rx_data)
        // Must match `DEVICE_CMD` in `LK_Device.py`
        8'hA1: command_from_rx_data = CMD_QUERY_FW_INFO;
        8'hA3: command_from_rx_data = CMD_SET_MODE_DMG;
        8'hA5: command_from_rx_data = CMD_SET_VOLTAGE_5V;
        8'hA6: command_from_rx_data = CMD_SET_VARIABLE;
        8'hA7: command_from_rx_data = CMD_SET_FLASH_CMD;
        8'hA8: command_from_rx_data = CMD_SET_ADDR_AS_INPUTS;
        8'hA9: command_from_rx_data = CMD_CLK_TOGGLE;
        8'hAC: command_from_rx_data = CMD_DISABLE_PULLUPS;
        8'hAD: command_from_rx_data = CMD_GET_VARIABLE;
        8'hAE: command_from_rx_data = CMD_GET_VAR_STATE;
        8'hAF: command_from_rx_data = CMD_SET_VAR_STATE;
        8'hB1: command_from_rx_data = CMD_DMG_CART_READ;
        8'hB2: command_from_rx_data = CMD_DMG_CART_WRITE;
        8'hB3: command_from_rx_data = CMD_DMG_CART_WRITE_SRAM;
        8'hB4: command_from_rx_data = CMD_DMG_MBC_RESET;
        8'hB8: command_from_rx_data = CMD_DMG_SET_BANK_CHANGE_CMD;
        8'hBA: command_from_rx_data = CMD_DMG_CART_READ_MEASURE;
        8'hD1: command_from_rx_data = CMD_DMG_FLASH_WRITE_BYTE;
        8'hD3: command_from_rx_data = CMD_FLASH_PROGRAM;
        8'hD4: command_from_rx_data = CMD_CART_WRITE_FLASH_CMD;
        8'hD5: command_from_rx_data = CMD_CALC_CRC32;
        8'hF5: command_from_rx_data = CMD_SET_PIN;
        default: ;
    endcase
end

command_t command = CMD_INIT;

wire en_init = command == CMD_INIT;
wire en_idle = command == CMD_IDLE;

wire en_query_fw_info = command == CMD_QUERY_FW_INFO;
wire cmd_query_fw_info_complete;
wire [7:0] cmd_query_fw_info_tx_data;
lk_cmd_query_fw_info_t cmd_query_fw_info(
    clk,
    ~en_query_fw_info,
    cmd_query_fw_info_complete,
    cmd_query_fw_info_tx_data
);

wire en_set_variable = command == CMD_SET_VARIABLE;
wire cmd_set_variable_complete;
vars_t cmd_set_variable_vars_out;
lk_cmd_set_variable_t cmd_set_variable_info(
    clk,
    ~en_set_variable,
    cmd_set_variable_complete,
    rx_valid,
    rx_data,
    vars,
    cmd_set_variable_vars_out
);


reg complete;
always @(*) begin
    complete = 1;
    unique case (1'b1)
        en_init: ;
        en_idle: complete = rx_valid;
        en_query_fw_info: complete = cmd_query_fw_info_complete;
        en_set_variable: complete = cmd_set_variable_complete;
    endcase
end

command_t next_command;
always @(*) begin
    next_command = command;
    if (reset) begin
        next_command = CMD_INIT;
    end else if (complete) begin
        next_command = en_idle ? command_from_rx_data : CMD_IDLE;
    end
end
always @(posedge clk) command <= next_command;

vars_t next_vars;
always @(*) begin
    next_vars = vars;
    if (reset) begin
        next_vars = '{default: 0};
    end else if (en_set_variable) next_vars = cmd_set_variable_vars_out;
end
always @(posedge clk) vars <= next_vars;


always @(posedge clk) begin
    tx_valid <= 1'b0;
    tx_data  <= 8'd0;
    if (reset) begin
        cart_enabled <= 1'b0;
    end else begin
        unique case (1'b1)
            en_init: begin
                tx_valid <= 1'b1;
                tx_data <= 8'hFF;
            end
            en_query_fw_info: begin
                // It sends a byte on each cycle, up to and including the cycle
                // when it completes
                tx_valid <= 1'b1;
                tx_data <= cmd_query_fw_info_tx_data;
            end
            en_set_variable: begin
                tx_valid <= complete;
                tx_data <= 1'b1;
            end
            default: ;
        endcase
    end
end

endmodule // cart_reader
`default_nettype wire
