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
    output reg         cart_rst,
    output reg         cart_data_dir_e,   // 1 = FPGA drives CART_D
    output reg  [7:0]  cart_d_out,        // data to write
    input  wire [7:0]  cart_d_in,         // data read from cart
    input  wire        cart_det,          // 0 = no cart (active-low)
    output reg         cart_pullups_enabled
);

// ============================================================
// Firmware variable indices (match DEVICE_VAR in LK_Device.py)
// ============================================================
// 32-bit vars (index 0-based within 32-bit array)
localparam VAR32_ADDRESS     = 2'd0;
localparam VAR32_AUTOPTOFF   = 2'd1;
// 16-bit vars
localparam VAR16_XFER_SIZE   = 3'd0;
localparam VAR16_BUF_SIZE    = 3'd1;
localparam VAR16_ROM_BANK    = 3'd2;
localparam VAR16_STATUS_REG  = 3'd3;
localparam VAR16_LAST_BANK   = 3'd4;
localparam VAR16_SR_MASK     = 3'd5;
localparam VAR16_SR_VALUE    = 3'd6;
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

// Firmware variables
reg [31:0] var32 [0:1];   // ADDRESS, AUTO_POWEROFF_TIME
reg [15:0] var16 [0:6];   // TRANSFER_SIZE … STATUS_REGISTER_VALUE
reg [7:0]  var8  [0:17];  // CART_MODE … DMG_AUDIO_ENABLED

// Convenience aliases
`define ADDRESS       var32[VAR32_ADDRESS]
`define XFER_SIZE     var16[VAR16_XFER_SIZE]
`define CART_MODE_V   var8[VAR8_CART_MODE]
`define ACCESS_MODE   var8[VAR8_ACCESS_MODE]
`define DMG_READ_CS_PULSE   var8[VAR8_DMG_READ_CS_PULSE][0]
`define DMG_WRITE_CS_PULSE  var8[VAR8_DMG_WRITE_CS_PULSE][0]


// ============================================================
// Protocol states
// ============================================================
localparam P_INIT        = 6'd0;   // Waiting for 0x55
localparam P_AA          = 6'd1;   // Got 0x55, waiting for 0xAA
localparam P_TX_ID       = 6'd2;   // Sending ID string
localparam P_HELLO_WAIT_L      = 6'd3;   // Waiting for 'L'
localparam P_HELLO_WAIT_K      = 6'd4;   // Got 'L', waiting for 'K'
localparam P_CMD         = 6'd6;   // Waiting for command byte
localparam P_TX_ACK      = 6'd7;   // Send 0x01, return to CMD
localparam P_TX_BYTES    = 6'd8;   // Send resp_buf[0..resp_len-1], return to CMD
localparam P_FW_INFO     = 6'd9;   // QUERY_FW_INFO multi-byte send
localparam P_SET_VAR_P   = 6'd10;  // SET_VARIABLE: collecting params
localparam P_GET_VAR_P   = 6'd11;  // GET_VARIABLE: collecting params
localparam P_GET_VAR_TX  = 6'd12;  // GET_VARIABLE: sending 4-byte result
localparam P_CART_RD_CHK = 6'd13;  // DMG_CART_READ: check cache
localparam P_CART_RD_TX  = 6'd15;  // Sending cached bytes to host
localparam P_CART_WR_P   = 6'd16;  // DMG_CART_WRITE: collecting 5 bytes
localparam P_CART_WR_DO  = 6'd17;  // DMG_CART_WRITE: doing the write
localparam P_SRAM_WR_RX  = 6'd18;  // DMG_CART_WRITE_SRAM: receiving data
localparam P_SRAM_WR_DO  = 6'd19;  // Performing SRAM write
localparam P_FLASH_RX    = 6'd20;  // FLASH_PROGRAM / SRAM WR: receive data byte
localparam P_FLASH_WR_DO = 6'd21;  // FLASH_PROGRAM: write one byte to cart
localparam P_FLB_WR_P    = 6'd22;  // DMG_FLASH_WRITE_BYTE: param collection
localparam P_FLB_WR_DO   = 6'd23;  // DMG_FLASH_WRITE_BYTE: write
localparam P_FLASH_CMD_P = 6'd24;  // CART_WRITE_FLASH_CMD: collecting header
localparam P_FLASH_CMD_E = 6'd25;  // CART_WRITE_FLASH_CMD: entry bytes
localparam P_FLASH_CMD_W = 6'd26;  // CART_WRITE_FLASH_CMD: write one entry
localparam P_CLK_TOG_P   = 6'd27;  // CLK_TOGGLE: collecting count
localparam P_CLK_TOG_DO  = 6'd28;  // CLK_TOGGLE: toggling
localparam P_SET_PIN_P   = 6'd29;  // SET_PIN: collecting 5 bytes
localparam P_GET_VAR_ST  = 6'd30;  // GET_VAR_STATE: sending all vars
localparam P_SET_VAR_ST  = 6'd31;  // SET_VAR_STATE: receiving (ignored)
localparam P_BYE_WAIT_L  = 6'd33;

// ============================================================
// Cart access states
// ============================================================
localparam C_IDLE  = 3'd0;
localparam C_SETUP = 3'd1;   // address stable, dir set
localparam C_CSRD  = 3'd2;   // CS/RD asserted
localparam C_WAIT  = 3'd3;   // hold
localparam C_DONE  = 3'd4;   // single-cycle done pulse
localparam C_WR_LOW = 3'd5;   // write: WR low
localparam C_WR_HOLD = 3'd6;
localparam C_WR_HIGH = 3'd7;   // write: WR high + drive data

// ============================================================
// Registers
// ============================================================
reg [5:0]  pstate;
reg [2:0]  cart_state;

// General parameter accumulator (up to 9 bytes for SET_VARIABLE)
reg [7:0]  par [0:8];
reg [3:0]  par_cnt;      // bytes remaining to collect
reg [3:0]  par_idx;      // index into par[]

// Response buffer (used by P_TX_BYTES, P_FW_INFO, P_GET_VAR_TX)
reg [7:0]  resp_buf [0:63];
reg [5:0]  resp_len;
reg [5:0]  resp_pos;

// ID string (sent after 0x55 0xAA)
// Adjust for release builds
localparam ID_STR = {"\0", "Chromatic FPGA FW L vYYYY.MM.DD.NN", "\r", "\0" };
localparam ID_LEN = $bits(ID_STR) / 8;
reg [5:0]  id_pos;

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

// FLASH_PROGRAM return code (0x01 normally, 0x03 if buffer still has room)
reg [7:0]  flash_ret;

// CART_WRITE_FLASH_CMD working registers
reg [7:0]  fcmd_num;      // number of entries remaining
reg [5:0]  fcmd_par_cnt;  // bytes in current entry remaining (6 each: addr32+val16)
reg [5:0]  fcmd_par_idx;
reg [7:0]  fcmd_par [0:5];

// CLK_TOGGLE counter
reg [31:0] clk_tog_cnt;

// GET_VAR_STATE / SET_VAR_STATE index
reg [6:0]  vstate_idx;
// Total bytes in var state dump: 2*4 + 7*2 + 18*1 = 8+14+18 = 40
localparam VSTATE_LEN = 40;

// SET_PIN receive counter
reg [2:0]  setpin_cnt;

// ============================================================
// ID string initialisation (combinational ROM)
// ============================================================
integer k;
initial begin
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

    for (k = 0; k < 2;  k = k+1) var32[k] = 32'd0;
    for (k = 0; k < 7;  k = k+1) var16[k] = 16'd0;
    for (k = 0; k < 18; k = k+1) var8[k]  = 8'd0;
end

// ============================================================
// SET_VARIABLE / GET_VARIABLE helpers
// ============================================================
// Key encoding: size(1 byte in par[0]) + key_id(4 bytes par[1..4] big-endian)
// Value: par[5] ignored (size 1 or 2 or 4 all packed into 4-byte LE field in par[5..8])
// Actually: SET_VARIABLE sends [size, key32_BE(4), value32_BE(4)] = 9 bytes total

task do_set_var;
    input [7:0] sz;
    input [31:0] key;
    input [31:0] val;
    begin
        case (sz)
        8'd4: begin
            // 32-bit vars keyed by 32-bit key (lower 8 bits used)
            case (key[7:0])
            8'h00: var32[VAR32_ADDRESS]   <= val;
            8'h01: var32[VAR32_AUTOPTOFF] <= val;
            default: ;
            endcase
        end
        8'd2: begin
            case (key[7:0])
            8'h00: var16[VAR16_XFER_SIZE]  <= val[15:0];
            8'h01: var16[VAR16_BUF_SIZE]   <= val[15:0];
            8'h02: var16[VAR16_ROM_BANK]   <= val[15:0];
            8'h03: var16[VAR16_STATUS_REG] <= val[15:0];
            8'h04: var16[VAR16_LAST_BANK]  <= val[15:0];
            8'h05: var16[VAR16_SR_MASK]    <= val[15:0];
            8'h06: var16[VAR16_SR_VALUE]   <= val[15:0];
            default: ;
            endcase
        end
        8'd1: begin
            case (key[7:0])
            8'h00: var8[VAR8_CART_MODE]   <= val[7:0];
            8'h01: var8[VAR8_ACCESS_MODE] <= val[7:0];
            8'h02: var8[VAR8_FLASH_CMDSET]<= val[7:0];
            8'h03: var8[VAR8_FLASH_METHOD]<= val[7:0];
            8'h04: var8[VAR8_FLASH_WEPIN] <= val[7:0];
            8'h05: var8[VAR8_FLASH_PULRST]<= val[7:0];
            8'h06: var8[VAR8_FLASH_CMDB1] <= val[7:0];
            8'h07: var8[VAR8_FLASH_SHRPSR]<= val[7:0];
            8'h08: var8[VAR8_DMG_READ_CS_PULSE] <= val[7:0];
            8'h09: var8[VAR8_DMG_WRITE_CS_PULSE] <= val[7:0];
            8'h0A: var8[VAR8_FLASH_DDIE]  <= val[7:0];
            8'h0B: var8[VAR8_DMG_RD_METH] <= val[7:0];
            8'h0C: var8[VAR8_AGB_RD_METH] <= val[7:0];
            8'h0D: var8[VAR8_CART_PWRD]   <= val[7:0];
            8'h0E: var8[VAR8_PULLUPS_EN]  <= val[7:0];
            8'h0F: var8[VAR8_AUTO_PWROFF] <= val[7:0];
            8'h10: var8[VAR8_AGB_IRQ_EN]  <= val[7:0];
            8'h11: var8[VAR8_DMG_AUD_EN]  <= val[7:0];
            default: ;
            endcase
        end
        default: ;
        endcase
    end
endtask

function [31:0] do_get_var;
    input [7:0] sz;
    input [31:0] key;
    begin
        do_get_var = 32'd0;
        case (sz)
        8'd4: begin
            case (key[7:0])
            8'h00: do_get_var = var32[VAR32_ADDRESS];
            8'h01: do_get_var = var32[VAR32_AUTOPTOFF];
            default: ;
            endcase
        end
        8'd2: begin
            case (key[7:0])
            8'h00: do_get_var = {16'd0, var16[VAR16_XFER_SIZE]};
            8'h01: do_get_var = {16'd0, var16[VAR16_BUF_SIZE]};
            8'h02: do_get_var = {16'd0, var16[VAR16_ROM_BANK]};
            8'h03: do_get_var = {16'd0, var16[VAR16_STATUS_REG]};
            8'h04: do_get_var = {16'd0, var16[VAR16_LAST_BANK]};
            8'h05: do_get_var = {16'd0, var16[VAR16_SR_MASK]};
            8'h06: do_get_var = {16'd0, var16[VAR16_SR_VALUE]};
            default: ;
            endcase
        end
        8'd1: begin
            case (key[7:0])
            8'h00: do_get_var = {24'd0, var8[VAR8_CART_MODE]};
            8'h01: do_get_var = {24'd0, var8[VAR8_ACCESS_MODE]};
            8'h02: do_get_var = {24'd0, var8[VAR8_FLASH_CMDSET]};
            8'h03: do_get_var = {24'd0, var8[VAR8_FLASH_METHOD]};
            8'h04: do_get_var = {24'd0, var8[VAR8_FLASH_WEPIN]};
            8'h05: do_get_var = {24'd0, var8[VAR8_FLASH_PULRST]};
            8'h06: do_get_var = {24'd0, var8[VAR8_FLASH_CMDB1]};
            8'h07: do_get_var = {24'd0, var8[VAR8_FLASH_SHRPSR]};
            8'h08: do_get_var = {24'd0, var8[VAR8_DMG_READ_CS_PULSE]};
            8'h09: do_get_var = {24'd0, var8[VAR8_DMG_WRITE_CS_PULSE]};
            8'h0A: do_get_var = {24'd0, var8[VAR8_FLASH_DDIE]};
            8'h0B: do_get_var = {24'd0, var8[VAR8_DMG_RD_METH]};
            8'h0C: do_get_var = {24'd0, var8[VAR8_AGB_RD_METH]};
            8'h0D: do_get_var = {24'd0, var8[VAR8_CART_PWRD]};
            8'h0E: do_get_var = {24'd0, var8[VAR8_PULLUPS_EN]};
            8'h0F: do_get_var = {24'd0, var8[VAR8_AUTO_PWROFF]};
            8'h10: do_get_var = {24'd0, var8[VAR8_AGB_IRQ_EN]};
            8'h11: do_get_var = {24'd0, var8[VAR8_DMG_AUD_EN]};
            default: ;
            endcase
        end
        default: ;
        endcase
    end
endfunction

// ============================================================
// Main state machine
// ============================================================
integer i;
reg [7:0]  cmd_r;           // current command byte
reg [31:0] get_var_result;  // result of GET_VARIABLE
reg [31:0] set_var_val;     // accumulated 32-bit value for SET_VARIABLE
reg [31:0] set_var_key;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        pstate          <= P_INIT;
        cart_state      <= C_IDLE;
        tx_valid        <= 1'b0;
        tx_data         <= 8'd0;
        cart_a          <= 16'hFFFF;
        cart_clk        <= 1'b1;
        cart_cs         <= 1'b1;
        cart_rd         <= 1'b1;
        cart_wr         <= 1'b1;
        cart_rst        <= 1'b1;
        cart_data_dir_e <= 1'b0;
        cart_d_out      <= 8'hFF;
        cart_done       <= 1'b0;
        flash_ret       <= 8'h01;
        cart_pullups_enabled <= 1'b0;
        for (i=0; i<2;  i=i+1) var32[i] <= 32'd0;
        for (i=0; i<7;  i=i+1) var16[i] <= 16'd0;
        for (i=0; i<18; i=i+1) var8[i]  <= 8'd0;
    end else begin
        // Defaults
        tx_valid  <= 1'b0;
        cart_done <= 1'b0;


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
            cart_wr  <= 1'b0;
            cart_cs  <= 1'b0;
            cart_clk <= 1'b0; // Drop clock with WR
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
                cart_wr       <= 1'b1; // De-assert WR
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
                cart_data_dir_e <= 1'b0;
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

        // ── Handshake ──────────────────────────────────────────────────
        P_INIT: begin
            if (rx_valid && rx_data == 8'h55)
                pstate <= P_AA;
        end

        P_AA: begin
            if (rx_valid) begin
                if (rx_data == 8'hAA) begin
                    id_pos <= 6'd0;
                    pstate <= P_TX_ID;
                end else begin
                    pstate <= P_INIT;
                end
            end
        end

        P_TX_ID: begin
            // Send ID string byte-by-byte
            if (!tx_valid) begin
                tx_data  <= ID_STR[(ID_LEN-1-id_pos) * 8 +: 8];
                tx_valid <= 1'b1;
                if (id_pos == ID_LEN[5:0] - 6'd1) begin
                    pstate <= P_HELLO_WAIT_L;
                end else begin
                    id_pos <= id_pos + 6'd1;
                end
            end
        end

        P_HELLO_WAIT_L: begin
            // Accept byte only after tx_valid deasserts (1-cycle pulse already sent)
            if (rx_valid) begin
                if (rx_data == "L")
                    pstate <= P_HELLO_WAIT_K;
                else
                    pstate <= P_INIT;   // anything else: reset
            end
        end

        P_HELLO_WAIT_K: begin
            if (rx_valid) begin
                if (rx_data == "K") begin
                    tx_data  <= 8'hFF;
                    tx_valid <= 1'b1;
                    pstate   <= P_CMD;
                end else begin
                    pstate <= P_INIT;
                end
            end
        end

        P_BYE_WAIT_L: begin
            if (rx_valid) begin
                if (rx_data == "L") begin
                    tx_data  <= 8'hFF;
                    tx_valid <= 1'b1;
                end
                pstate <= P_INIT;
            end
        end

        // ── Main command dispatcher ─────────────────────────────────────
        P_CMD: begin
            if (rx_valid) begin
                cmd_r     <= rx_data;
                par_idx   <= 3'd0;
                par_cnt   <= 3'd0;

                case (rx_data)
                // ─ Bye ("KL") ───────────────────────────────────────
                "K": pstate <= P_BYE_WAIT_L;
                // ─ Simple ACK commands ──────────────────────────────
                8'h43, // OFW_CART_MODE
                8'hA2, // SET_MODE_AGB   → ACK (not really supported)
                8'hA3, // SET_MODE_DMG
                8'hA4, // SET_VOLTAGE_3_3V
                8'hA5, // SET_VOLTAGE_5V
                8'hA7, // SET_FLASH_CMD (not used)
                8'hA8, // SET_ADDR_AS_INPUTS
                8'hC9, // AGB_BOOTUP_SEQUENCE
                8'hD5, // CALC_CRC32 (stub)
                8'hF1, // BOOTLOADER_RESET
                8'hF2, // CART_PWR_ON
                8'hF3: // CART_PWR_OFF
                begin
                    pstate <= P_TX_ACK;
                end

                8'hB4: begin // DMG_MBC_RESET
                    cart_a          <= 16'h0000;
                    cart_d_out      <= 8'h00;
                    cart_data_dir_e <= 1'b1;
                    cart_write_r    <= 1'b1;
                    cart_state      <= C_SETUP;
                    pstate          <= P_CART_WR_DO;
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
                    tx_data  <= 8'h01;
                    tx_valid <= 1'b1;
                    pstate   <= P_CMD;
                end

                8'hA1: begin // QUERY_FW_INFO
                    fwi_pos <= 5'd0;
                    pstate  <= P_FW_INFO;
                end

                8'hA6: begin // SET_VARIABLE: size(1) + key(4) + value(4) = 9 bytes
                    par_cnt <= 4'd8; // collect 9 bytes at indices 0..8
                    par_idx <= 4'd0;
                    pstate  <= P_SET_VAR_P;
                end

                8'hAD: begin // GET_VARIABLE: size(1) + key(4) = 5 bytes
                    par_cnt <= 4'd5;
                    par_idx <= 4'd0;
                    pstate  <= P_GET_VAR_P;
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
                    par_cnt <= 4'd4;
                    par_idx <= 4'd0;
                    pstate  <= P_CLK_TOG_P;
                end

                8'hF5, // SET_PIN: 4-byte mask + 1-byte direction = 5 bytes
                8'hB8: // DMG_SET_BANK_CHANGE_CMD: 3 bytes (cmd+bank1+bank2)
                begin
                    par_cnt <= (rx_data == 8'hF5) ? 4'd5 : 4'd3;
                    par_idx <= 4'd0;
                    pstate  <= P_SET_PIN_P;
                end

                8'hB1, // DMG_CART_READ
                8'hBA: // DMG_CART_READ_MEASURE (same as READ for us)
                begin
                    pstate <= P_CART_RD_CHK;
                end

                8'hB2: begin // DMG_CART_WRITE: addr(4 BE) + val(1) = 5 bytes
                    par_cnt <= 4'd5;
                    par_idx <= 4'd0;
                    pstate  <= P_CART_WR_P;
                end

                8'hD1: begin // DMG_FLASH_WRITE_BYTE: addr(4 BE) + val(1) = 5 bytes
                    par_cnt <= 4'd5;
                    par_idx <= 4'd0;
                    pstate  <= P_FLB_WR_P;
                end

                8'hB3: begin // DMG_CART_WRITE_SRAM: receive XFER_SIZE bytes then write
                    xfer_remain <= `XFER_SIZE;
                    pstate      <= P_SRAM_WR_RX;
                end

                8'hD3: begin // FLASH_PROGRAM: receive XFER_SIZE bytes, write each
                    xfer_remain <= `XFER_SIZE;
                    flash_ret   <= 8'h01;
                    pstate      <= P_FLASH_RX;
                end

                8'hD4: begin // CART_WRITE_FLASH_CMD: flashcart(1) + num(1) + entries
                    par_cnt  <= 4'd2;   // read 2 header bytes
                    par_idx  <= 4'd0;
                    pstate   <= P_FLASH_CMD_P;
                end

                default: begin
                    // Unknown command: ACK
                    pstate <= P_TX_ACK;
                end
                endcase
            end
        end // P_CMD

        // ── Send single ACK 0x01 ────────────────────────────────────────
        P_TX_ACK: begin
            if (!tx_valid && cart_state == C_IDLE) begin
                tx_data  <= 8'h01;
                tx_valid <= 1'b1;
                pstate   <= P_CMD;
            end
        end

        // ── Send multi-byte response ────────────────────────────────────
        P_TX_BYTES: begin
            if (!tx_valid) begin
                tx_data  <= resp_buf[resp_pos];
                tx_valid <= 1'b1;
                if (resp_pos == resp_len - 6'd1) begin
                    pstate <= P_CMD;
                end else begin
                    resp_pos <= resp_pos + 6'd1;
                end
            end
        end

        // ── QUERY_FW_INFO ───────────────────────────────────────────────
        P_FW_INFO: begin
            if (!tx_valid) begin
                tx_data  <= fwi_buf[fwi_pos];
                tx_valid <= 1'b1;
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
        P_SET_VAR_P: begin
            if (rx_valid) begin
                par[par_idx] <= rx_data;
                if (par_cnt == 4'd0) begin
                    // 9th byte received (value LSB); par[5..7]=val[31:8], rx_data=val[7:0]
                    do_set_var(
                        par[0],
                        {par[1], par[2], par[3], par[4]},
                        {par[5], par[6], par[7], rx_data}
                    );
                    pstate <= P_TX_ACK;
                end else begin
                    par_cnt <= par_cnt - 4'd1;
                    par_idx <= par_idx + 4'd1;
                end
            end
        end

        // ── GET_VARIABLE: read size(1)+key(4) = 5 bytes ─────────────────
        P_GET_VAR_P: begin
            if (rx_valid) begin
                par[par_idx] <= rx_data;
                if (par_cnt == 4'd1) begin
                    // All 5 bytes received: par[0]=size, par[1..3]=key[31:8], rx_data=key[7:0]
                    get_var_result <= do_get_var(
                        par[0],                          // size
                        {par[1], par[2], par[3], rx_data} // key BE
                    );
                    resp_pos <= 6'd0;
                    pstate <= P_GET_VAR_TX;
                end else begin
                    par_cnt <= par_cnt - 4'd1;
                    par_idx <= par_idx + 4'd1;
                end
            end
        end

        P_GET_VAR_TX: begin
            // Send 4 bytes big-endian
            if (!tx_valid) begin
                case (resp_pos[1:0])
                2'd0: tx_data <= get_var_result[31:24];
                2'd1: tx_data <= get_var_result[23:16];
                2'd2: tx_data <= get_var_result[15:8];
                2'd3: tx_data <= get_var_result[7:0];
                endcase
                tx_valid <= 1'b1;
                if (resp_pos[1:0] == 2'd3) begin
                    resp_pos <= 6'd0;
                    pstate   <= P_CMD;
                end else begin
                    resp_pos <= resp_pos + 6'd1;
                end
            end
        end

        // ── GET_VAR_STATE: dump all variables ──────────────────────────
        P_GET_VAR_ST: begin
            if (!tx_valid) begin
                // Emit bytes in order: var32[0] BE, var32[1] BE, var16[0..6] BE, var8[0..17]
                tx_valid <= 1'b1;
                if (vstate_idx < 8) begin
                    // 2 × 32-bit (8 bytes)
                    case (vstate_idx[2:0])
                    3'd0: tx_data <= var32[0][31:24];
                    3'd1: tx_data <= var32[0][23:16];
                    3'd2: tx_data <= var32[0][15:8];
                    3'd3: tx_data <= var32[0][7:0];
                    3'd4: tx_data <= var32[1][31:24];
                    3'd5: tx_data <= var32[1][23:16];
                    3'd6: tx_data <= var32[1][15:8];
                    3'd7: tx_data <= var32[1][7:0];
                    endcase
                end else if (vstate_idx < 22) begin
                    // 7 × 16-bit (14 bytes), starting at index 8
                    case (vstate_idx[3:0] - 4'd8)
                    4'd0:  tx_data <= var16[0][15:8];
                    4'd1:  tx_data <= var16[0][7:0];
                    4'd2:  tx_data <= var16[1][15:8];
                    4'd3:  tx_data <= var16[1][7:0];
                    4'd4:  tx_data <= var16[2][15:8];
                    4'd5:  tx_data <= var16[2][7:0];
                    4'd6:  tx_data <= var16[3][15:8];
                    4'd7:  tx_data <= var16[3][7:0];
                    4'd8:  tx_data <= var16[4][15:8];
                    4'd9:  tx_data <= var16[4][7:0];
                    4'd10: tx_data <= var16[5][15:8];
                    4'd11: tx_data <= var16[5][7:0];
                    4'd12: tx_data <= var16[6][15:8];
                    4'd13: tx_data <= var16[6][7:0];
                    default: tx_data <= 8'd0;
                    endcase
                end else begin
                    // 18 × 8-bit (18 bytes), starting at index 22
                    tx_data <= var8[vstate_idx - 7'd22];
                end
                vstate_idx <= vstate_idx + 7'd1;
                if (vstate_idx == VSTATE_LEN[6:0] - 7'd1)
                    pstate <= P_CMD;
            end
        end

        // ── SET_VAR_STATE: receive all variables (we ignore the data) ───
        P_SET_VAR_ST: begin
            if (rx_valid) begin
                vstate_idx <= vstate_idx + 7'd1;
                if (vstate_idx == VSTATE_LEN[6:0] - 7'd1)
                    pstate <= P_CMD;
            end
        end

        // ── CLK_TOGGLE ──────────────────────────────────────────────────
        P_CLK_TOG_P: begin
            if (rx_valid) begin
                par[par_idx] <= rx_data;
                if (par_cnt == 4'd1) begin
                    clk_tog_cnt <= {par[0], par[1], par[2], rx_data};
                    pstate <= P_CLK_TOG_DO;
                end else begin
                    par_cnt <= par_cnt - 4'd1;
                    par_idx <= par_idx + 4'd1;
                end
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

        // ── SET_PIN / DMG_SET_BANK_CHANGE_CMD: receive N bytes then ACK ─
        P_SET_PIN_P: begin
            if (rx_valid) begin
                if (par_cnt == 4'd1) begin
                    pstate <= P_TX_ACK;
                end else begin
                    par_cnt <= par_cnt - 4'd1;
                    par_idx <= par_idx + 4'd1;
                end
            end
        end

        // ── DMG_CART_READ / DMG_CART_READ_MEASURE ──────────────────────
        P_CART_RD_CHK: begin
            xfer_remain <= `XFER_SIZE;
            cart_a          <= `ADDRESS[15:0];
            cart_data_dir_e <= 1'b0;
            cart_write_r    <= 1'b0;
            cart_state      <= C_SETUP;
            pstate          <= P_CART_RD_TX;
        end


        P_CART_RD_TX: begin
            // SRAM path: wait for cart access to complete
            if (cart_done) begin
                tx_data  <= cart_din_r;
                tx_valid <= 1'b1;
                `ADDRESS    <= `ADDRESS + 32'd1;
                xfer_remain <= xfer_remain - 16'd1;
                if (xfer_remain == 16'd1) begin
                    pstate <= P_CMD;
                end else begin
                    // Kick off next byte
                    cart_a          <= `ADDRESS[15:0] + 16'd1;
                    cart_data_dir_e <= 1'b0;
                    cart_write_r    <= 1'b0;
                    cart_state      <= C_SETUP;
                end
            end
        end

        // ── DMG_CART_WRITE: addr(4 BE) + val(1) ────────────────────────
        P_CART_WR_P: begin
            if (rx_valid) begin
                par[par_idx] <= rx_data;
                if (par_cnt == 4'd1) begin
                    // All 5 bytes received: par[0..3]=addr BE, rx_data=val
                    cart_a          <= {par[2], par[3]}; // lower 16 bits of address
                    cart_d_out      <= rx_data;
                    cart_data_dir_e <= 1'b1;
                    cart_write_r    <= 1'b1;
                    cart_state      <= C_SETUP;
                    pstate          <= P_CART_WR_DO;
                end else begin
                    par_cnt <= par_cnt - 4'd1;
                    par_idx <= par_idx + 4'd1;
                end
            end
        end

        P_CART_WR_DO: begin
            if (cart_done) begin
                pstate <= P_TX_ACK;
            end
        end

        // ── DMG_CART_WRITE_SRAM: receive XFER_SIZE bytes, write each ───
        P_SRAM_WR_RX: begin
            if (rx_valid) begin
                cart_a          <= `ADDRESS[15:0];
                cart_d_out      <= rx_data;
                cart_data_dir_e <= 1'b1;
                cart_write_r    <= 1'b1;
                cart_state      <= C_SETUP;
                pstate          <= P_SRAM_WR_DO;
            end
        end

        P_SRAM_WR_DO: begin
            if (cart_done) begin
                `ADDRESS    <= `ADDRESS + 32'd1;
                xfer_remain <= xfer_remain - 16'd1;
                if (xfer_remain == 16'd1) begin
                    pstate <= P_TX_ACK;
                end else begin
                    pstate <= P_SRAM_WR_RX;
                end
            end
        end

        // ── FLASH_PROGRAM: receive XFER_SIZE bytes, write each ─────────
        P_FLASH_RX: begin
            if (rx_valid) begin
                cart_a          <= `ADDRESS[15:0];
                cart_d_out      <= rx_data;
                cart_data_dir_e <= 1'b1;
                cart_write_r    <= 1'b1;
                cart_state      <= C_SETUP;
                pstate          <= P_FLASH_WR_DO;
            end
        end

        P_FLASH_WR_DO: begin
            if (cart_done) begin
                `ADDRESS    <= `ADDRESS + 32'd1;
                xfer_remain <= xfer_remain - 16'd1;
                if (xfer_remain == 16'd1) begin
                    // Send ACK 0x01
                    pstate <= P_TX_ACK;
                end else begin
                    pstate <= P_FLASH_RX;
                end
            end
        end

        // ── DMG_FLASH_WRITE_BYTE: addr(4 BE) + val(1) ──────────────────
        P_FLB_WR_P: begin
            if (rx_valid) begin
                par[par_idx] <= rx_data;
                if (par_cnt == 4'd1) begin
                    cart_a          <= {par[2], par[3]};
                    cart_d_out      <= rx_data;
                    cart_data_dir_e <= 1'b1;
                    cart_write_r    <= 1'b1;
                    cart_state      <= C_SETUP;
                    pstate          <= P_CART_WR_DO;  // reuse, ACKs after done
                end else begin
                    par_cnt <= par_cnt - 4'd1;
                    par_idx <= par_idx + 4'd1;
                end
            end
        end

        // ── CART_WRITE_FLASH_CMD ────────────────────────────────────────
        P_FLASH_CMD_P: begin
            // Receive flashcart_flag(1) + num_entries(1)
            if (rx_valid) begin
                par[par_idx] <= rx_data;
                if (par_cnt == 4'd1) begin
                    // par[0]=flashcart, rx_data=num
                    fcmd_num     <= rx_data;
                    fcmd_par_cnt <= 6'd6;  // 4-byte addr + 2-byte val
                    fcmd_par_idx <= 6'd0;
                    if (rx_data == 8'd0) begin
                        pstate <= P_TX_ACK;   // no entries
                    end else begin
                        pstate <= P_FLASH_CMD_E;
                    end
                end else begin
                    par_cnt <= par_cnt - 4'd1;
                    par_idx <= par_idx + 4'd1;
                end
            end
        end

        P_FLASH_CMD_E: begin
            // Receive 6 bytes per entry: addr(4 BE) + val(2 BE)
            if (rx_valid) begin
                fcmd_par[fcmd_par_idx[2:0]] <= rx_data;
                if (fcmd_par_cnt == 6'd1) begin
                    // Entry complete
                    fcmd_addr <= {fcmd_par[2], fcmd_par[3]};  // lower 16 bits
                    fcmd_val  <= fcmd_par[4];                  // lower 8 bits used
                    cart_a          <= {fcmd_par[2], fcmd_par[3]};
                    cart_d_out      <= fcmd_par[4];
                    cart_data_dir_e <= 1'b1;
                    cart_write_r    <= 1'b1;
                    cart_state      <= C_SETUP;
                    pstate          <= P_FLASH_CMD_W;
                end else begin
                    fcmd_par_cnt <= fcmd_par_cnt - 6'd1;
                    fcmd_par_idx <= fcmd_par_idx + 6'd1;
                end
            end
        end

        P_FLASH_CMD_W: begin
            if (cart_done) begin
                fcmd_num <= fcmd_num - 8'd1;
                if (fcmd_num == 8'd1) begin
                    pstate <= P_TX_ACK;
                end else begin
                    fcmd_par_cnt <= 6'd6;
                    fcmd_par_idx <= 6'd0;
                    pstate       <= P_FLASH_CMD_E;
                end
            end
        end

        default: pstate <= P_INIT;
        endcase // pstate

    end // ~reset
end // always

endmodule // cart_reader
`default_nettype wire
