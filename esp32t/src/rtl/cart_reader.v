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
//   0xAF (SET_VAR_STATE)  → receive all vars (ignored)
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

module cart_reader #(
    parameter CLK_FREQ     = 60_000_000,
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
    output reg         lk_disable = 1'b0,

    // Parallel byte interface (EP3 via usbuvcuart_top)
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    output reg         tx_valid,
    output reg  [7:0]  tx_data,

    // Cartridge bus (top-level drives tristate from these)
    output reg  [15:0] cart_a,
    output reg         cart_clk,
    output reg         cart_cs,
    output reg         cart_rd,
    output reg         cart_wr,
    output reg         cart_rst_out,
    output reg         cart_data_dir_e,   // 1 = read, 0 = write
    output reg  [7:0]  cart_d_out,        // data to write
    input  wire [7:0]  cart_d_in,         // data read from cart
    output reg         cart_audio,
    input  wire        cart_det,          // 0 = no cart (active-low)
    output reg         cart_pullups_enabled
);

// ============================================================
// Firmware variable indices (match DEVICE_VAR in LK_Device.py)
// ============================================================
// 32-bit vars (index 0-based within 32-bit array)
localparam VAR32_ADDRESS     = 1'd0;
localparam VAR32_AUTOPTOFF   = 1'd1;
localparam VAR32__MAX        = 1'd1;
localparam VAR32__COUNT      = VAR32__MAX + 1;
// 16-bit vars
localparam VAR16_XFER_SIZE   = 3'd0;
localparam VAR16_BUF_SIZE    = 3'd1;
localparam VAR16_ROM_BANK    = 3'd2;
localparam VAR16_STATUS_REG  = 3'd3;
localparam VAR16_LAST_BANK   = 3'd4;
localparam VAR16_SR_MASK     = 3'd5;
localparam VAR16_SR_VALUE    = 3'd6;
localparam VAR16__MAX        = 3'd6;
localparam VAR16__COUNT      = VAR16__MAX + 1;
// 8-bit vars
localparam VAR8_CART_MODE    = 5'd0;
localparam VAR8_ACCESS_MODE  = 5'd1;
localparam VAR8_FLASH_CMDSET = 5'd2;
localparam VAR8_FLASH_METHOD = 5'd3;
localparam VAR8_FLASH_WEPIN  = 5'd4;
localparam VAR8_FLASH_PULRST = 5'd5;
localparam VAR8_FLASH_CMDB1  = 5'd6;
localparam VAR8_FLASH_SHRPSR = 5'd7;
localparam VAR8_DMG_READ_CS_PULSE  = 5'd8;
localparam VAR8_DMG_WRITE_CS_PULSE  = 5'd9;
localparam VAR8_FLASH_DDIE   = 5'd10;
localparam VAR8_DMG_RD_METH  = 5'd11;
localparam VAR8_AGB_RD_METH  = 5'd12;
localparam VAR8_CART_PWRD    = 5'd13;
localparam VAR8_PULLUPS_EN   = 5'd14;
localparam VAR8_AUTO_PWROFF  = 5'd15;
localparam VAR8_AGB_IRQ_EN   = 5'd16;
localparam VAR8_DMG_AUD_EN   = 5'd17;
localparam VAR8__MAX         = 5'd17;
localparam VAR8__COUNT       = VAR8__MAX + 1;

localparam VAR__COUNT        = VAR32__COUNT + VAR16__COUNT + VAR8__COUNT;
localparam VAR__BYTES        = VAR__COUNT << 2;

logic [((VAR__BYTES * 8) - 1):0] fw_vars;

localparam VAR32__OFFSET = 0;
localparam VAR16__OFFSET = VAR32__OFFSET + VAR32__COUNT;
localparam VAR8__OFFSET  = VAR16__OFFSET + VAR16__COUNT;

`define VAR32_SLICE(key, count)   fw_vars[(VAR32__OFFSET + key) * 32 +: count]
`define VAR16_SLICE(key, count)   fw_vars[(VAR16__OFFSET + key) * 32 +: count]
`define VAR8_SLICE(key, count)    fw_vars[(VAR8__OFFSET  + key) * 32 +: count]
`define VAR32(key) `VAR32_SLICE(key, 32)
`define VAR16(key) `VAR16_SLICE(key, 16)
`define VAR8(key)  `VAR8_SLICE(key, 8)

// Convenience aliases
`define ADDRESS            `VAR32_SLICE(VAR32_ADDRESS, 16)
`define XFER_SIZE          `VAR16(VAR16_XFER_SIZE)
`define CART_MODE_V        `VAR8(VAR8_CART_MODE)
`define ACCESS_MODE        `VAR8(VAR8_ACCESS_MODE)
`define DMG_READ_CS_PULSE  `VAR8_SLICE(VAR8_DMG_READ_CS_PULSE, 1)
`define DMG_WRITE_CS_PULSE `VAR8_SLICE(VAR8_DMG_WRITE_CS_PULSE, 1)
`define FLASH_WE_PIN       `VAR8_SLICE(VAR8_FLASH_WEPIN, 1)

// These are 16-bit variables, but on DMG we only have an 8-bit data bus
// Assuming 16-bit is for GBA support
`define STATUS_REGISTER_MASK  `VAR16_SLICE(VAR16_SR_MASK, 8)
`define STATUS_REGISTER_VALUE `VAR16_SLICE(VAR16_SR_VALUE, 8)
`define STATUS_REGISTER       `VAR16_SLICE(VAR16_STATUS_REG, 8)

localparam CART_WRITE_PULSE_PINS_NONE = 2'd0;
localparam CART_WRITE_PULSE_PINS_WR = 2'd1;
localparam CART_WRITE_PULSE_PINS_AUDIO = 2'd2;
localparam CART_WRITE_PULSE_PINS_DEFAULT = 2'd3;
reg [2:0] cart_write_pulse_pins = CART_WRITE_PULSE_PINS_DEFAULT;

// ============================================================
// Protocol states
// ============================================================
typedef enum {
    P_CMD, // Waiting for command byte
    P_TX_ACK, // Send 0x01, return to CMD
    P_TX_BYTES, // Send from tx_bytes_idx to tx_bytes_count , return to CMD
    P_FW_INFO, // QUERY_FW_INFO multi-byte send
    P_GET_VAR_INIT,
    P_GET_VAR_P, // GET_VARIABLE: collecting params
    P_SET_VAR_INIT,
    P_SET_VAR_P, // SET_VARIABLE: collecting params
    P_CART_RD, // DMG_CART_READ
    P_CART_RD_TX, // Sending cached bytes to host
    P_CART_WR_INIT,
    P_CART_WR_P, // DMG_CART_WRITE: collecting 5 bytes
    P_CART_WR_DO, // DMG_CART_WRITE: doing the write
    P_SRAM_WR_RX, // DMG_CART_WRITE_SRAM: receiving data
    P_SRAM_WR_DO, // Performing SRAM write
    P_SRAM_WR_WAIT, // Wait for SRAM write to finish
    P_FLASH_PROGRAM_RX, // FLASH_PROGRAM / SRAM WR: receive data byte
    P_FLASH_PROGRAM_WR_COMMANDS, // populate flash_program_commands for one data byte
    P_FLASH_PROGRAM_WR_DO, // write one command byte to cart
    P_FLASH_PROGRAM_WR_WAIT_WRITE, // Wait for `cart_done` on the actual right, then switch to reading the status
    P_FLASH_PROGRAM_WR_WAIT_STATUS, // Wait for (status & mask == value), then go back to P_CALC_FLASH_WR_DO
    P_FLB_WR_INIT,
    P_FLB_WR_P, // DMG_FLASH_WRITE_BYTE: param collection
    P_FLB_WR_DO, // DMG_FLASH_WRITE_BYTE: write
    P_CART_WRITE_FLASH_CMD_INIT,
    P_CART_WRITE_FLASH_CMD_P, // CART_WRITE_FLASH_CMD: collecting header
    P_CART_WRITE_FLASH_CMD_E, // CART_WRITE_FLASH_CMD: entry bytes
    P_CART_WRITE_FLASH_CMD_W, // CART_WRITE_FLASH_CMD: write one entry
    P_CART_WRITE_FLASH_CMD_W_NOWAIT,
    P_CART_WRITE_FLASH_CMD_WAIT_STATUS,
    P_CLK_TOG_INIT,
    P_CLK_TOG_P, // CLK_TOGGLE: collecting count
    P_CLK_TOG_DO, // CLK_TOGGLE: toggling
    P_SET_PIN_INIT,
    P_SET_PIN_P, // SET_PIN: collecting 5 bytes
    P_GET_VAR_ST, // GET_VAR_STATE: sending all vars
    P_SET_VAR_ST, // SET_VAR_STATE: receiving (ignored)
    P_SET_FLASH_CMD_INIT,
    P_SET_FLASH_CMD_P,
    P_SET_FLASH_CMD_E,
    P_SET_FLASH_CMD_UPDATE_COUNT,
    P_SET_BANK_CHANGE_CMD_INIT,
    P_SET_BANK_CHANGE_CMD_P,
    P_SET_BANK_CHANGE_CMD_E,
    P_CALC_CRC_INIT,
    P_CALC_CRC_P,
    P_CALC_CRC_RD,
    P_CALC_CRC_DO,
    P_BYE_WAIT_L // Got 'L', waiting for 'L'
} pstate_t;


// ============================================================
// Cart access states
// ============================================================
typedef enum logic [3:0] {
    C_IDLE,
    C_SETUP,  // address stable, dir set
    C_CSRD,   // CS/RD asserted
    C_WAIT,   // hold
    C_DONE,   // single-cycle done pulse
    C_WR_LOW, // write: WR low
    C_WR_HOLD,
    C_WR_HIGH // write: WR high + drive data
} cart_state_t;

// ============================================================
// Registers
// ============================================================
pstate_t     pstate;
cart_state_t cart_state;

// 16kb buffer for writes, CRC, or other large operations
localparam BLOB_SIZE = 16 * 1024;
reg [7:0]  blob [0:BLOB_SIZE - 1];
localparam BLOB_IDX_WIDTH = $clog2(BLOB_SIZE);
reg [(BLOB_IDX_WIDTH - 1):0] blob_idx;

// FW info bytes (sent after QUERY_FW_INFO 0xA1)
// Format: size(1)=0x08, then 8 bytes: cfw_id='L'(1), fw_ver=12(2BE), pcb_ver=0x42(1), fw_ts(4BE)
// Then: name_len(1), name(N), cart_power_ctrl(1)=0, bootloader_reset(1)=0
localparam FWI_LEN = 26;
reg [7:0]  fwi_buf [0:FWI_LEN-1];
reg [4:0]  fwi_pos;

// Cart access working registers
reg [15:0] cart_addr_r;   // current 16-bit cart address
reg [7:0]  cart_dout_r;   // byte to write
reg        cart_write_r;  // 1=write, 0=read
reg [7:0]  cart_din_r;    // latched read result
reg        cart_done;     // pulses for one cycle when cart access complete
reg [4:0]  cart_wait_cnt;

// Transfer counters
reg [15:0] xfer_remain;   // bytes remaining in current transfer
reg [13:0] send_offset;   // offset within cache for current send

// SET_FLASH_CMD working registers
localparam FLASH_COMMANDS_MAX = 6;
typedef struct packed {
    logic [15:0] unused_addr_msb; // Only for GBA
    logic [15:0] address;
    logic [7:0] unused_data_msb; // Only for GBA
    logic [7:0] data;
} flash_command_t;

union packed {
  flash_command_t [0:FLASH_COMMANDS_MAX - 1] as_struct;
  logic [0:(FLASH_COMMANDS_MAX * ($bits(flash_command_t) / 8)) - 1][7:0] as_bytes;
} flash_commands;
reg [($clog2(FLASH_COMMANDS_MAX + 1) - 1):0] flash_command_count; // total number of flash commands
reg [($clog2($bits(flash_commands)+ 1) - 1):0] flash_command_rx_idx;
reg [($clog2($bits(flash_commands)+ 1) - 1):0] flash_command_do_idx;
reg [7:0] flash_command_written_data;

// CART_WRITE_FLASH_CMD working registers
reg [1:0]  fcmd_p_idx;
reg [7:0]  fcmd_entry_count; // number of entries remaining
reg [7:0]  fcmd_par [0:6*16];
reg [2:0]  fcmd_idx;

// GET_VAR_STATE / SET_VAR_STATE index
reg [$clog2(VAR__BYTES) - 1:0]  vstate_idx;

// SET_PIN receive counter
reg [2:0]  set_pin_p_idx;

// DMT_SET_BANK_CHANGE_CMD variables
reg [7:0] set_bank_count;

// DMG_CART_WRITE
reg [15:0] dmg_cart_write_a;
reg [2:0]  dmg_cart_write_p_idx;

// DMG_FLASH_WRITE_BYTE
reg [2:0]  flb_wr_p_idx;
reg [15:0] flb_wr_a;

// CALC_CRC32 working registers
reg [2:0]  crc_p_idx;
reg [15:0] crc_idx;
reg [15:0] crc_remaining;
reg [31:0] crc_state;

function [31:0] next_crc;
    input [31:0] current_crc;
    input [7:0]  data;
    reg   [31:0] crc;
    integer j;
    begin
        crc = current_crc ^ {24'b0, data};
        for (j = 0; j < 8; j = j + 1) begin
            if (crc[0]) crc = (crc >> 1) ^ 32'hEDB88320;
            else        crc = (crc >> 1);
        end
        next_crc = crc;
    end
endfunction

// CLK_TOGGLE registers
reg [1:0]  clk_tog_p_idx;
reg [31:0] clk_tog_cnt;


// TX mapping

reg [7:0] tx_bytes_idx;
reg [7:0] tx_bytes_count;
enum logic [3:0] {
    TXS_NONE,
    TXS_CONSTANT_ONE,
    TXS_CONSTANT_FF,
    TXS_CALC_CRC,
    TXS_GET_VAR,
    TXS_CART_IN,
    TXS_FW_INFO
} tx_data_sel;
always_comb begin
    case (tx_data_sel)
        TXS_NONE: begin
            tx_data = 8'h00;
        end
        TXS_CONSTANT_ONE: begin
            tx_data = 8'h11;
        end
        TXS_CONSTANT_FF: begin
            tx_data = 8'hFF;
        end
        TXS_CALC_CRC: begin
            tx_data = crc_state[tx_bytes_idx[1:0] * 8 +: 8];
        end
        TXS_GET_VAR: begin
            tx_data = set_get_var_data.as_bytes[tx_bytes_idx];
        end
        TXS_CART_IN: begin
            tx_data = cart_din_r;
        end
        TXS_FW_INFO: begin
            tx_data = fwi_buf[fwi_pos];
        end
    endcase
end

// ============================================================
// ID string initialisation (combinational ROM)
// ============================================================
integer k;
initial begin
    cart_audio = 1'b0;
    // FW info buffer
    // size=8
    fwi_buf[0]  = 8'd8;
    // cfw_id = 'L'  (uses LK protocol, but pcb_ver 0x42 ∉ Joey-Jr PCB_VERSIONS)
    fwi_buf[1]  = "L";
    // fw_ver = 12  (big-endian 16-bit)
    fwi_buf[2]  = 8'd0;
    fwi_buf[3]  = 8'd12;
    // pcb_ver = 0x42  (not in Joey-Jr's PCB_VERSIONS → rejected by hw_JoeyJr.py)
    fwi_buf[4]  = 8'h42;
    // fw_ts = 0x69FB3C8C
    fwi_buf[5]  = 8'h69;
    fwi_buf[6]  = 8'hFB;
    fwi_buf[7]  = 8'h3C;
    fwi_buf[8]  = 8'h8C;
    // name_len = 14  ("Chromatic Cart")
    fwi_buf[9]  = 8'd14;
    fwi_buf[10] = "C";
    fwi_buf[11] = "h";
    fwi_buf[12] = "r";
    fwi_buf[13] = "o";
    fwi_buf[14] = "m";
    fwi_buf[15] = "a";
    fwi_buf[16] = "t";
    fwi_buf[17] = "i";
    fwi_buf[18] = "c";
    fwi_buf[19] = " ";
    fwi_buf[20] = "C";
    fwi_buf[21] = "a";
    fwi_buf[22] = "r";
    fwi_buf[23] = "t";
    // cart_power_ctrl = 0
    fwi_buf[24] = 8'd0;
    // bootloader_reset = 0
    fwi_buf[25] = 8'd0;

    fw_vars = 0;
end

// ============================================================
// SET_VARIABLE / GET_VARIABLE helpers
// ============================================================
// Key encoding: size(1 byte in par[0]) + key_id(4 bytes par[1..4] big-endian)
// Value: par[5] ignored (size 1 or 2 or 4 all packed into 4-byte LE field in par[5..8])
// Actually: SET_VARIABLE sends [size, key32_BE(4), value32_BE(4)] = 9 bytes total

reg [3:0]  set_var_par_idx;
reg [2:0]  get_var_par_idx;

reg [7:0]  set_get_var_size;
reg [7:0]  set_get_var_key;
union packed {
    logic [31:0]       as_bits;
    logic [0:3]  [7:0] as_bytes;
} set_get_var_data;
reg [31:0] set_get_var_data;
reg [$clog2(VAR__COUNT + 1) - 1:0] set_get_var_idx;

reg [1:0] get_var_tx_idx;

function automatic logic [$clog2(VAR__COUNT + 1) - 1:0] var32_idx(logic [7:0] sz, logic[7:0] k);
    case (sz)
        8'd4: return (k <= VAR32__MAX) ? (VAR32__OFFSET + k) : 0;
        8'd2: return (k <= VAR16__MAX) ? (VAR16__OFFSET + k) : 0;
        8'd1: return (k <= VAR8__MAX) ? (VAR8__OFFSET + k) : 0;
        default: return 0;
    endcase
endfunction

// ============================================================
// Main state machine
// ============================================================
integer i;
reg cmd_rcv;

always @(posedge clk) begin
    tx_valid <= 1'b0;
    if (reset) begin
        pstate          <= P_CMD;
        cart_state      <= C_IDLE;
        tx_valid        <= 1'b0;
        cart_a          <= 16'hFFFF;
        cart_clk        <= 1'b1;
        cart_cs         <= 1'b1;
        cart_rd         <= 1'b1;
        cart_wr         <= 1'b1;
        cart_rst_out    <= 1'b1;
        cart_data_dir_e <= 1'b1;
        cart_d_out      <= 8'hFF;
        cart_done       <= 1'b0;
        cart_pullups_enabled <= 1'b0;

        fw_vars <= 0;
        tx_data_sel <= TXS_NONE;
        cmd_rcv <= 1'b0;
    end else begin
        // Defaults
        cart_done <= 1'b0;
        lk_disable <= 1'b0;
        cmd_rcv <= 1'b0;

        // ─────────────────────────────────────────────────────────────────
        // Cart access state machine (runs every cycle, driven by pstate)
        // ─────────────────────────────────────────────────────────────────
        case (cart_state)
        C_IDLE: ; // nothing

        C_SETUP: begin
            // Address and direction already set by caller one cycle ago.
            // Now assert CS with setup delay.
            cart_wait_cnt <= CART_SETUP[4:0] - 5'd1;
            cart_state <= C_CSRD;
        end

        C_CSRD: begin
            if (cart_wait_cnt != 0) begin
                cart_wait_cnt <= cart_wait_cnt - 5'd1;
            end else begin
                if (cart_write_r) begin
                    if (`DMG_WRITE_CS_PULSE) cart_cs <= 1'b0;
                    cart_wait_cnt <= CART_WR_HOLD[4:0] - 5'd1;
                    cart_state    <= C_WR_LOW;
                end else begin
                    if (`DMG_READ_CS_PULSE) cart_cs <= 1'b0;
                    cart_rd       <= 1'b0;
                    cart_wait_cnt <= CART_RD_HOLD[4:0] - 5'd1;
                    cart_state    <= C_WAIT;
                end
            end
        end

        C_WAIT: begin
            if (cart_wait_cnt != 0) begin
                cart_wait_cnt <= cart_wait_cnt - 5'd1;
            end else begin
                cart_din_r <= cart_d_in;
                cart_rd    <= 1'b1;
                cart_cs    <= 1'b1;
                cart_state <= C_DONE;
            end
        end

        C_WR_LOW: begin
            case (cart_write_pulse_pins)
                CART_WRITE_PULSE_PINS_DEFAULT: begin
                    cart_wr <= 1'b0;
                    cart_clk <= 1'b0;
                end
                CART_WRITE_PULSE_PINS_WR: cart_wr <= 1'b0;
                CART_WRITE_PULSE_PINS_AUDIO: cart_audio <= 1'b0;
                CART_WRITE_PULSE_PINS_NONE: begin
                end
            endcase

            if (cart_wait_cnt != 0) begin
                cart_wait_cnt <= cart_wait_cnt - 5'd1;
            end else begin
                cart_clk      <= 1'b1; // Raise clock WHILE WR is low
                cart_wait_cnt <= CART_WR_HOLD[4:0] - 5'd1;
                cart_state    <= C_WR_HOLD;
            end
        end

        C_WR_HOLD: begin
            // WR and CS are still low here
            if (cart_wait_cnt != 0) begin
                cart_wait_cnt <= cart_wait_cnt - 5'd1;
            end else begin
                case (cart_write_pulse_pins)
                  CART_WRITE_PULSE_PINS_DEFAULT, CART_WRITE_PULSE_PINS_WR: cart_wr <= 1'b1;
                  CART_WRITE_PULSE_PINS_AUDIO: cart_audio <= 1'b1;
                  CART_WRITE_PULSE_PINS_NONE: begin
                  end
                endcase
                cart_cs       <= 1'b1; // De-assert CS
                cart_wait_cnt <= CART_WR_HOLD[4:0] - 5'd1;
                cart_state    <= C_WR_HIGH;
            end
        end

        C_WR_HIGH: begin
            // Hold data/address stable for a moment after WR goes high
            if (cart_wait_cnt != 0) begin
                cart_wait_cnt <= cart_wait_cnt - 5'd1;
            end else begin
                cart_data_dir_e <= 1'b1;
                cart_state      <= C_DONE;
            end
        end

        C_DONE: begin
            cart_done  <= 1'b1;
            cart_state <= C_IDLE;
        end
        endcase // cart_state

        // ─────────────────────────────────────────────────────────────────
        // Protocol state machine
        // ─────────────────────────────────────────────────────────────────
        case (pstate)

        P_BYE_WAIT_L: begin
            if (rx_valid) begin
                pstate <= P_CMD;
                if (rx_data == "L") begin
                    tx_data_sel <= TXS_CONSTANT_FF;
                    tx_valid <= 1'b1;
                    lk_disable <= 1'b1;
                end
            end
        end

        // ── Main command dispatcher ─────────────────────────────────────
        P_CMD: begin
            if (rx_valid) begin
                cmd_rcv   <= 1'b1;
                tx_data_sel <= TXS_NONE;

                case (rx_data)
                // ─ Bye ("KL") ───────────────────────────────────────
                "K": pstate <= P_BYE_WAIT_L;
                // ─ Stub ACK commands ────────────────────────────────
                8'hA3, // SET_MODE_DMG
                8'hA5, // SET_VOLTAGE_5V
                8'hA8, // SET_ADDR_AS_INPUTS
                8'hF1, // BOOTLOADER_RESET
                8'hF2, // CART_PWR_ON
                8'hF3: // CART_PWR_OFF
                begin
                    pstate <= P_TX_ACK;
                end

                // ─ Stub commands without an ACK ───────────────────────
                8'hA2, // SET_MODE_AGB (should be ACKed, but unsupported)
                8'hC9, // AGB_BOOTUP_SEQUENCE (ditto)
                8'hA4, // SET_VOLTAGE_3_3V (ditto)
                8'h43: begin // OFW_CART_MODE (unused)
                    pstate <= P_CMD;
                end

                8'hB4: begin // DMG_MBC_RESET
                    cart_a          <= 16'h0000;
                    cart_d_out      <= 8'h00;
                    cart_data_dir_e <= 1'b0;
                    cart_write_r    <= 1'b1;
                    cart_state      <= C_SETUP;
                    pstate          <= P_CART_WR_DO;
                end

                8'hD5: begin // CALC_CRC32
                    pstate <= P_CALC_CRC_INIT;
                end

                8'hA7: begin // SET_FLASH_CMD
                    pstate <= P_SET_FLASH_CMD_INIT;
                end

                8'hAB: begin // ENABLE_PULLUPS
                    cart_pullups_enabled <= 1'b1;
                    pstate <= P_TX_ACK;
                end

                8'hAC: begin // DISABLE_PULLUPS
                    cart_pullups_enabled <= 1'b0;
                    pstate <= P_TX_ACK;
                end

                8'hF4: begin // QUERY_CART_PWR → respond 0x01 (always on)
                    tx_data_sel <= TXS_CONSTANT_ONE;
                    tx_valid <= 1'b1;
                    pstate   <= P_CMD;
                end

                8'hA1: begin // QUERY_FW_INFO
                    fwi_pos <= 5'd0;
                    pstate  <= P_FW_INFO;
                end

                8'hA6: begin // SET_VARIABLE: size(1) + key(4) + value(4) = 9 bytes
                    pstate  <= P_SET_VAR_INIT;
                end

                8'hAD: begin // GET_VARIABLE: size(1) + key(4) = 5 bytes
                    pstate  <= P_GET_VAR_INIT;
                end

                8'hAE: begin // GET_VAR_STATE
                    vstate_idx <= 7'd0;
                    pstate <= P_GET_VAR_ST;
                end

                8'hAF: begin // SET_VAR_STATE: receive VSTATE_LEN bytes (ignored)
                    vstate_idx <= 7'd0;
                    pstate <= P_SET_VAR_ST;
                end

                8'hA9: begin // CLK_TOGGLE: count(4 bytes BE)
                    pstate  <= P_CLK_TOG_INIT;
                end

                8'hF5: // SET_PIN: 4-byte mask + 1-byte direction = 5 bytes
                begin
                    pstate  <= P_SET_PIN_INIT;
                end

                8'hB8: begin // DMG_SET_BANK_CHANGE_CMD: 1-byte count, then N * (4-byte value/address, 1-byte type)
                    pstate <= P_SET_BANK_CHANGE_CMD_P;
                end

                8'hB1, // DMG_CART_READ
                8'hBA: // DMG_CART_READ_MEASURE (same as READ for us)
                begin
                    pstate <= P_CART_RD;
                end

                8'hB2: begin // DMG_CART_WRITE: addr(4 BE) + val(1) = 5 bytes
                    pstate  <= P_CART_WR_INIT;
                end

                8'hD1: begin // DMG_FLASH_WRITE_BYTE: addr(4 BE) + val(1) = 5 bytes
                    pstate  <= P_FLB_WR_INIT;
                end

                8'hB3: begin // DMG_CART_WRITE_SRAM: receive XFER_SIZE bytes then write
                    xfer_remain <= `XFER_SIZE;
                    blob_idx    <= 0;
                    pstate      <= P_SRAM_WR_RX;
                end

                8'hD3: begin // FLASH_PROGRAM: receive XFER_SIZE bytes, write each
                    xfer_remain <= `XFER_SIZE;
                    blob_idx <= 16'd0;
                    pstate      <= P_FLASH_PROGRAM_RX;
                end

                8'hD4: begin // CART_WRITE_FLASH_CMD: flashcart(1) + num(1) + entries
                    pstate   <= P_CART_WRITE_FLASH_CMD_INIT;
                end

                default: begin
                    // Unknown command: ACK
                    pstate <= P_TX_ACK;
                end
                endcase
            end
        end // P_CMD

        P_CALC_CRC_INIT: begin
            crc_p_idx <= 3'd0;
            crc_idx   <= 16'd0;
            crc_state <= ~32'd0;
            pstate <= P_CALC_CRC_P;
        end

        P_CALC_CRC_P: begin
            // We have `ADDRESS already set via SET_FW_VARIABLE
            // Now we need to get the 4-byte BE chunk length
            if (rx_valid) begin
                case (crc_p_idx)
                    3'd0, 3'd1: /* GBA-only MSB */ ;
                    3'd2, 3'd3: crc_remaining <= { crc_remaining[15:0], rx_data };
                endcase
                if (crc_p_idx == 3'd3) begin
                    pstate <= P_CALC_CRC_RD;
                end else crc_p_idx <= crc_p_idx + 1;
            end
        end

        P_CALC_CRC_RD: begin
            cart_a <= `ADDRESS + crc_idx + (cart_done ? 16'd1 : 16'd0);
            cart_data_dir_e <= 1'b1;
            cart_write_r <= 1'b0;
            cart_state <= C_SETUP;
            pstate <= P_CALC_CRC_DO;
        end

        P_CALC_CRC_DO: begin
            logic last_byte;

            last_byte = (crc_idx == crc_remaining - 16'd1);
            pstate <= P_CALC_CRC_DO;

            if (cart_done) begin
                if (last_byte) begin
                    // Like most LK commands, FlashGBX does the MB <-> LE conversion
                    crc_state <= next_crc(crc_state, cart_din_r) ^ 32'hFFFFFFFF;

                    tx_data_sel    <= TXS_CALC_CRC;
                    tx_bytes_idx   <= 0;
                    tx_bytes_count <= 4;
                    pstate         <= P_TX_BYTES;
                end else begin
                    crc_state <= next_crc(crc_state, cart_din_r);
                    crc_idx   <= crc_idx + 16'd1;
                    pstate    <= P_CALC_CRC_RD;
                end
            end
        end

        P_SET_BANK_CHANGE_CMD_P: begin
            if (rx_valid) begin
                // 1-byte count, then n 5-byte entires
                if (rx_data == 8'd0) begin
                    pstate <= P_TX_ACK;
                end else begin
                    set_bank_count <= rx_data * 8'd5;
                    pstate <= P_SET_BANK_CHANGE_CMD_E;
                end
            end
        end

        P_SET_BANK_CHANGE_CMD_E: begin
            // TODO: stub - just ignores the command
            // This is fine for the MBC3000v4 as it has no commands
            if (rx_valid) begin
              set_bank_count <= set_bank_count - 8'd1;
              if (set_bank_count == 8'd1) pstate <= P_TX_ACK;
            end
        end

        P_SET_FLASH_CMD_INIT: begin
            flash_command_rx_idx <= 0;
            pstate <= P_SET_FLASH_CMD_P;
        end

        P_SET_FLASH_CMD_P: begin
            // 3 bytes: command set, method (buffered/unbuffered/... weird), pins.
            // We only support no-command-set (direct writes), unbuffered
            // TODO: byte 3 is 'pins'. For now, we just treat this as a set of
            // `FLASH_WE_PIN`; however we don't support 4 == WR_RESET, which conflicts
            // with 'DEFAULT' for the override register. However, mapping 'unsupported'
            // to default seems reasonable for now.
            if (rx_valid) begin
                if (flash_command_rx_idx == 2) begin
                    `FLASH_WE_PIN        <= rx_data;
                    flash_command_count  <= FLASH_COMMANDS_MAX;
                    flash_command_rx_idx <= 0;
                    pstate <= P_SET_FLASH_CMD_E;
                end else begin
                    flash_command_rx_idx <= flash_command_rx_idx + 1;
                end
            end
        end

        P_SET_FLASH_CMD_E: begin
            if (rx_valid) begin
                flash_commands.as_bytes[flash_command_rx_idx] <= rx_data;

                if (flash_command_rx_idx == ($bits(flash_commands.as_bytes) / 8) - 1) begin
                    pstate <= P_SET_FLASH_CMD_UPDATE_COUNT;
                end else flash_command_rx_idx <= flash_command_rx_idx + 1;
            end
        end

        P_SET_FLASH_CMD_UPDATE_COUNT: begin
            // We enter this with flash_command_count == FLASH_COMMANDS_MAX
            for (integer i = 0; i < FLASH_COMMANDS_MAX; i = i + 1) begin
                if (flash_commands.as_struct[i].address == 16'h0000 && flash_commands.as_struct[i].data == 16'h0000) begin
                    flash_command_count <= i;
                    break;
                end
            end
            pstate <= P_TX_ACK;
        end

        // ── Send single ACK 0x01 ────────────────────────────────────────
        P_TX_ACK: begin
            if (!tx_valid) begin
                tx_valid    <= 1'b1;
                tx_data_sel <= TXS_CONSTANT_FF;
                pstate      <= P_CMD;
            end
        end

        P_TX_BYTES: begin
            if (!tx_valid) begin
                tx_valid <= 1'b1;
                tx_bytes_idx <= tx_bytes_idx + 1'b1;
                if (tx_bytes_idx == tx_bytes_count - 1'b1) begin
                    pstate <= P_CMD;
                end
            end
        end

        // ── QUERY_FW_INFO ───────────────────────────────────────────────
        P_FW_INFO: begin
            if (!tx_valid) begin
                tx_data_sel <= TXS_FW_INFO;
                tx_valid    <= 1'b1;
                if (fwi_pos == FWI_LEN[4:0] - 5'd1) begin
                    pstate <= P_CMD;
                end else begin
                    fwi_pos <= fwi_pos + 5'd1;
                end
            end
        end

        // ── SET_VARIABLE: read size(1)+key(4)+value(4) = 9 bytes ───────
        // par[0]=size, par[1..4]=key BE, par[5..8]=value BE
        // par_cnt starts at 8 and counts down; fires when par_cnt==0 (9th byte).
        /*
        P_SET_VAR_INIT: begin
            set_var_par_idx <= 0;
            pstate <= P_SET_VAR_P;
        end
        P_SET_VAR_P: begin
            if (rx_valid) begin
                case (set_var_par_idx)
                    4'd0: set_get_var_size <= rx_data;
                    4'd1, 4'd2, 4'd3: /* unused key MSB * / ;
                    4'd4: set_get_var_key <= rx_data;
                    4'd5, 4'd6, 4'd7:
                        set_get_var_data.as_bits <= { set_get_var_data.as_bits[23:0], rx_data};
                    4'd8: begin
                        `VAR32(var32_idx(set_get_var_size, set_get_var_key)) <= { set_get_var_data.as_bits[23:0], rx_data };
                        pstate <= P_TX_ACK;
                    end
                endcase
                set_var_par_idx <= set_var_par_idx + 4'd1;
            end
        end

        // ── GET_VARIABLE: read size(1)+key(4) = 5 bytes ─────────────────
        P_GET_VAR_INIT: begin
            get_var_par_idx <= 0;
            pstate <= P_GET_VAR_P;
        end
        P_GET_VAR_P: begin
            if (rx_valid) begin
                case (get_var_par_idx)
                    3'd0: set_get_var_size <= rx_data;
                    3'd1, 3'd2, 3'd3: /* unused key MSB * / ;
                    3'd4: begin
                        set_get_var_key <= rx_data;
                        tx_data_sel     <= TXS_GET_VAR;
                        tx_bytes_idx    <= 0;
                        tx_bytes_count  <= 4;
                        pstate          <= P_TX_BYTES;
                    end
                endcase
                get_var_par_idx <= get_var_par_idx + 3'd1;
            end
        end
        */

        /* FIXME

        // ── GET_VAR_STATE: dump all variables ──────────────────────────
        P_GET_VAR_ST: begin
            if (!tx_valid) begin
                // Emit bytes in order: var32[0] BE, var32[1] BE, var16[0..6] BE, var8[0..17]
                tx_valid <= 1'b1;
                tx_data <= fw_vars[vstate_idx * 8 +: 8];
                vstate_idx <= vstate_idx + 7'd1;
                if (vstate_idx == VAR__BYTES - 1'd1) pstate <= P_CMD;
            end
        end

        // ── SET_VAR_STATE: receive all variables ───
        P_SET_VAR_ST: begin
            if (rx_valid) begin
                fw_vars[vstate_idx * 8 +: 8] <= rx_data;
                vstate_idx <= vstate_idx + 7'd1;
                if (vstate_idx == VAR__BYTES - 1'd1)
                    pstate <= P_CMD;
            end
        end
        FIXME */

        // ── CLK_TOGGLE ──────────────────────────────────────────────────
        P_CLK_TOG_INIT: begin
            clk_tog_p_idx <= 0;
            pstate <= P_CLK_TOG_P;
        end

        P_CLK_TOG_P: begin
            if (rx_valid) begin
                clk_tog_cnt <= {clk_tog_cnt[23:0], rx_data};
                if (clk_tog_p_idx == 3) begin
                    pstate <= P_CLK_TOG_DO;
                end else clk_tog_p_idx <= clk_tog_p_idx + 1;
            end
        end

        P_CLK_TOG_DO: begin
            if (clk_tog_cnt != 0) begin
                cart_clk    <= ~cart_clk;
                clk_tog_cnt <= clk_tog_cnt - 32'd1;
            end else begin
                pstate <= P_TX_ACK;
            end
        end

        // ── SET_PIN: 4-byte mask + 1-byte direction = 5 bytes
        P_SET_PIN_INIT: begin
            set_pin_p_idx <= 0;
            pstate <= P_SET_PIN_P;
        end

        // TODO: this is a stub
        P_SET_PIN_P: begin
            if (rx_valid) begin
                if (set_pin_p_idx == 4) begin
                    pstate <= P_TX_ACK;
                end else set_pin_p_idx <= set_pin_p_idx + 1;
            end
        end

        // ── DMG_CART_READ / DMG_CART_READ_MEASURE ──────────────────────
        P_CART_RD: begin
            xfer_remain <= `XFER_SIZE;
            cart_a          <= `ADDRESS;
            cart_data_dir_e <= 1'b1;
            cart_write_r    <= 1'b0;
            cart_state      <= C_SETUP;
            pstate          <= P_CART_RD_TX;
        end


        P_CART_RD_TX: begin
            // SRAM path: wait for cart access to complete
            if (cart_done) begin
                tx_data_sel <= TXS_CART_IN;
                tx_valid <= 1'b1;
                `ADDRESS    <= `ADDRESS + 32'd1;
                xfer_remain <= xfer_remain - 16'd1;
                if (xfer_remain == 16'd1) begin
                    pstate <= P_CMD;
                end else begin
                    // Kick off next byte
                    cart_a          <= `ADDRESS + 16'd1;
                    cart_data_dir_e <= 1'b1;
                    cart_write_r    <= 1'b0;
                    cart_state      <= C_SETUP;
                end
            end
        end

        // ── DMG_CART_WRITE: addr(4 BE) + val(1) ────────────────────────
        P_CART_WR_INIT: begin
            dmg_cart_write_p_idx <= 0;
            pstate <= P_CART_WR_P;
        end

        P_CART_WR_P: begin
            if (rx_valid) begin
                // All 5 bytes received: par[0..3]=addr BE, rx_data=val
                case (dmg_cart_write_p_idx)
                    3'd0, 3'd1: /* ignore, address MSB on GBA */ ;
                    3'd2, 3'd3: begin
                        dmg_cart_write_a = { dmg_cart_write_a[7:0], rx_data };
                    end
                    3'd4: begin
                        cart_a          <= dmg_cart_write_a;
                        cart_d_out      <= rx_data;
                        cart_data_dir_e <= 1'b0;
                        cart_write_r    <= 1'b1;
                        cart_state      <= C_SETUP;
                        pstate          <= P_CART_WR_DO;
                    end
                endcase
                dmg_cart_write_p_idx <= dmg_cart_write_p_idx + 1;
            end
        end

        P_CART_WR_DO: begin
            if (cart_done) begin
                cart_write_pulse_pins <= CART_WRITE_PULSE_PINS_DEFAULT;
                pstate <= P_TX_ACK;
            end
        end

        // ── DMG_CART_WRITE_SRAM: receive XFER_SIZE bytes, write each ───
        P_SRAM_WR_RX: begin
            if (rx_valid) begin
                blob[blob_idx] <= rx_data;
                if (blob_idx == `XFER_SIZE - 1) begin
                    blob_idx <= 0;
                    pstate   <= P_SRAM_WR_DO;
                end else begin
                    blob_idx <= blob_idx + 1;
                end
            end
        end

        P_SRAM_WR_DO: begin
                cart_a          <= `ADDRESS;
                cart_d_out      <= blob[blob_idx];
                cart_data_dir_e <= 1'b0;
                cart_write_r    <= 1'b1;
                cart_state      <= C_SETUP;
                pstate          <= P_SRAM_WR_WAIT;
        end

        P_SRAM_WR_WAIT: begin
            if (cart_done) begin
                `ADDRESS <= `ADDRESS + 16'd1;
                if (blob_idx == `XFER_SIZE - 1) begin
                    pstate <= P_TX_ACK;
                end else begin
                    blob_idx <= blob_idx + 1;
                    pstate <= P_SRAM_WR_DO;
                end
            end
        end

        // ── FLASH_PROGRAM: receive XFER_SIZE bytes, write each ─────────
        P_FLASH_PROGRAM_RX: begin
            if (rx_valid) begin
                blob[blob_idx] <= rx_data;
                xfer_remain <= xfer_remain - 16'd1;
                if (xfer_remain == 16'd1) begin
                    blob_idx <= 16'd0;
                    pstate <= P_FLASH_PROGRAM_WR_COMMANDS;
                end else blob_idx <= blob_idx + 16'd1;
            end
        end

        P_FLASH_PROGRAM_WR_COMMANDS: begin
            flash_commands.as_struct[flash_command_count].address <= `ADDRESS + blob_idx;
            flash_commands.as_struct[flash_command_count].data <= blob[blob_idx];
            flash_command_written_data <= blob[blob_idx];

            flash_command_do_idx <= 0;
            pstate <= P_FLASH_PROGRAM_WR_DO;
        end

        P_FLASH_PROGRAM_WR_DO: begin
            cart_write_pulse_pins <= `FLASH_WE_PIN;
            cart_a <= flash_commands.as_struct[flash_command_do_idx].address;
            cart_d_out <= flash_commands.as_struct[flash_command_do_idx].data;
            cart_data_dir_e <= 1'b0;
            cart_write_r <= 1'b1;
            cart_state <= C_SETUP;

            pstate <= P_FLASH_PROGRAM_WR_WAIT_WRITE;
        end

        P_FLASH_PROGRAM_WR_WAIT_WRITE: begin
            if (cart_done) begin
                if (flash_command_do_idx == flash_command_count) begin
                    // After the queued commands, we have an extra one that's the actual data
                    cart_data_dir_e <= 1'b1;
                    cart_write_r <= 1'b0;
                    cart_state <= C_SETUP;
                    pstate <= P_FLASH_PROGRAM_WR_WAIT_STATUS;
                end else begin
                    flash_command_do_idx <= flash_command_do_idx + 1;
                    pstate <= P_FLASH_PROGRAM_WR_DO;
                end
            end
        end

        P_FLASH_PROGRAM_WR_WAIT_STATUS: begin
            logic last_byte;
            last_byte = (blob_idx == `XFER_SIZE - 16'd1);

            if (cart_done) begin
                `STATUS_REGISTER <= cart_d_in;
                if (cart_d_in[7] == flash_command_written_data[7]) begin
                    if (last_byte) begin
                        cart_write_pulse_pins <= CART_WRITE_PULSE_PINS_DEFAULT;
                        pstate <= P_TX_ACK;
                        // LK_Device::WriteROM only sets ADDRESS on the first chunk,
                        // so we need to increment it before the next one.
                        `ADDRESS <= `ADDRESS + `XFER_SIZE;
                    end else begin
                        blob_idx <= blob_idx + 1;
                        pstate <= P_FLASH_PROGRAM_WR_COMMANDS;
                    end
                end else cart_state <= C_SETUP;
            end
        end

        // ── DMG_FLASH_WRITE_BYTE: addr(4 BE) + val(1) ──────────────────
        P_FLB_WR_INIT: begin
            flb_wr_p_idx <= 0;
            pstate <= P_FLB_WR_P;
        end
        P_FLB_WR_P: begin
            if (rx_valid) begin
                case (flb_wr_p_idx)
                    3'd0, 3'd1: /* GBA addr MSB */ ;
                    3'd2, 3'd3: begin
                        flb_wr_a = { flb_wr_a[7:0], rx_data};
                    end
                    3'd4: begin
                        cart_write_pulse_pins <= `FLASH_WE_PIN;
                        cart_a          <= flb_wr_a;
                        cart_d_out      <= rx_data;
                        cart_data_dir_e <= 1'b0;
                        cart_write_r    <= 1'b1;
                        cart_state      <= C_SETUP;
                        pstate          <= P_CART_WR_DO;  // reuse, ACKs after done
                    end
                endcase
                flb_wr_p_idx <= flb_wr_p_idx + 1;
            end
        end

        // ── CART_WRITE_FLASH_CMD ────────────────────────────────────────
        // Receive flashcart_flag(1) + num_entries(1)

        P_CART_WRITE_FLASH_CMD_INIT: begin
            fcmd_p_idx <= 0;
            pstate <= P_CART_WRITE_FLASH_CMD_P;
        end

        P_CART_WRITE_FLASH_CMD_P: begin
            if (rx_valid) begin
                case (fcmd_p_idx)
                    2'd0: /* flashcart [UNUSED] */ ;
                    2'd1: begin
                        fcmd_entry_count <= rx_data;
                        if (rx_data == 0) begin
                            pstate <= P_TX_ACK;
                        end else begin
                            fcmd_idx <= 8'd0;
                            pstate <= P_CART_WRITE_FLASH_CMD_E;
                        end
                    end
                endcase
                fcmd_p_idx <= fcmd_p_idx + 1;
            end
        end

        P_CART_WRITE_FLASH_CMD_E: begin
            // Receive 6 bytes per entry: addr(4 BE) + val(2 BE)
            if (rx_valid) begin
                fcmd_par[fcmd_idx] <= rx_data;
                if (fcmd_idx == (6 * fcmd_entry_count) - 1) begin
                    fcmd_idx <= 0;
                    pstate <= P_CART_WRITE_FLASH_CMD_W_NOWAIT;
                end else begin
                    fcmd_idx <= fcmd_idx + 8'd1;
                end
            end
        end

        P_CART_WRITE_FLASH_CMD_W, P_CART_WRITE_FLASH_CMD_W_NOWAIT: begin
            if (cart_done || pstate == P_CART_WRITE_FLASH_CMD_W_NOWAIT) begin
                if (fcmd_entry_count == 8'd0) begin
                    cart_write_pulse_pins <= CART_WRITE_PULSE_PINS_DEFAULT;
                    cart_data_dir_e <= 1'b1;
                    cart_write_r <= 1'b0;
                    cart_state <= C_SETUP;
                    pstate <= P_FLASH_PROGRAM_WR_WAIT_STATUS;
                    pstate <= P_CART_WRITE_FLASH_CMD_WAIT_STATUS;
                end else begin
                    // Entry complete
                    // DMG only has 16-bit addresses
                    cart_a          <= {fcmd_par[fcmd_idx+2], fcmd_par[fcmd_idx+3]};
                    // entry[4]: MSB
                    // entry[5]: LSB, on next cycle
                    // Ignore MSB: it is unused for DMG, only for AGB
                    // rx_data also contains the LSB, and is available this cycle
                    cart_d_out      <= fcmd_par[fcmd_idx+5];
                    cart_data_dir_e <= 1'b0;
                    cart_write_r    <= 1'b1;
                    cart_write_pulse_pins <= `FLASH_WE_PIN;
                    cart_state      <= C_SETUP;

                    fcmd_entry_count <= fcmd_entry_count - 8'd1;
                    fcmd_idx <= fcmd_idx + 8'd6;
                    pstate <= P_CART_WRITE_FLASH_CMD_W;
                end
            end
        end

        P_CART_WRITE_FLASH_CMD_WAIT_STATUS: begin
            if (cart_done) begin
                `STATUS_REGISTER <= cart_d_in;
                pstate <= P_TX_ACK;
            end
        end

        default: pstate <= P_CMD;
        endcase // pstate

    end // ~reset
end // always

endmodule // cart_reader
`default_nettype wire
