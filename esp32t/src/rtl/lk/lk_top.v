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

module lk_top #(
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
    input reg          lk_enabled,
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
    input  wire        cart_det           // 0 = no cart (active-low)
);

reg vars_reset = 0;

wire var_dmg_read_cs_pulse;
wire var_dmg_write_cs_pulse;

wire       vars_tx_valid;
wire [7:0] vars_tx_data;
wire       vars_cmd_complete;

lk_vars_t vars(
    .clk(clk),
    .reset(vars_reset),
    .get_variable_en(command == CMD_GET_VARIABLE),
    .set_variable_en(command == CMD_SET_VARIABLE),
    .complete(vars_cmd_complete),
    .rx_valid(rx_valid),
    .rx_data(rx_data),
    .tx_valid(vars_tx_valid),
    .tx_data(vars_tx_data),
    .var_dmg_read_cs_pulse(var_dmg_read_cs_pulse),
    .var_dmg_write_cs_pulse(var_dmg_read_cs_pulse)
);

enum {
    CART_WRITE_PULSE_PINS_NONE,
    CART_WRITE_PULSE_PINS_WR,
    CART_WRITE_PULSE_PINS_AUDIO,
    CART_WRITE_PULSE_PINS_DEFAULT
} cart_write_pulse_pins = CART_WRITE_PULSE_PINS_DEFAULT;

// ============================================================
// Protocol states
// ============================================================
// We can't use an enum as we want to be able to directly assign from rx_data in cmd_idle_t
typedef reg[7:0] command_t;
localparam CMD_IDLE = 8'h00;
localparam CMD_QUERY_FW_INFO = 8'hA1;
localparam CMD_SET_MODE_DMG = 8'hA3;
localparam CMD_SET_VOLTAGE_5V = 8'hA5;
localparam CMD_SET_VARIABLE = 8'hA6;
localparam CMD_SET_FLASH_CMD = 8'hA7;
localparam CMD_CLK_TOGGLE = 8'hA9;
localparam CMD_DISABLE_PULLUPS = 8'hAC;
localparam CMD_GET_VARIABLE = 8'hAD;
localparam CMD_GET_VAR_STATE = 8'hAE;
localparam CMD_SET_VAR_STATE = 8'hAF;
localparam CMD_DMG_CART_READ = 8'hB1;
localparam CMD_DMG_CART_WRITE = 8'hB2;
localparam CMD_DMG_CART_WRITE_SRAM = 8'hB3;
localparam CMD_DMG_MBC_RESET = 8'hB4;
localparam CMD_DMG_SET_BANK_CHANGE_CMD = 8'hB8;
localparam CMD_DMG_CART_READ_MEASURE = 8'hBA;
localparam CMD_DMG_FLASH_WRITE_BYTE = 8'hD1;
localparam CMD_FLASH_PROGRAM = 8'hD3;
localparam CMD_CART_WRITE_FLASH_CMD = 8'hD4;
localparam CMD_CALC_CRC32 = 8'hD5;
localparam CMD_SET_PIN = 8'hF5;

// ============================================================
// Cart access states
// ============================================================
typedef enum logic [3:0] {
    C_IDLE,
    C_SETUP,  // address stable, dir set
    C_READ_PULSE_VAR,
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
command_t command = CMD_IDLE;
cart_state_t cart_state;
wire cart_complete = (cart_state == C_DONE);


// Cart access working registers
reg [7:0]  cart_din_r;    // latched read result
reg        cart_done;     // pulses for one cycle when cart access complete
reg [4:0]  cart_wait_cnt;

// Transfer counters
reg [15:0] xfer_remain;   // bytes remaining in current transfer

// SET_FLASH_CMD working registers
localparam FLASH_COMMANDS_MAX = 6;
// 16-bit address and 8-bit data for DMG; the protocol is double the size as while we don't support
// AGB, the protocol does.
typedef struct packed {
    logic [15:0] address;
    logic [7:0] data;
} flash_command_t;

logic [15:0] flash_command_address;
flash_command_t [0:FLASH_COMMANDS_MAX - 1] flash_commands;
reg [($clog2(FLASH_COMMANDS_MAX + 1) - 1):0] flash_command_count; // total number of flash commands
reg [($clog2($bits(flash_commands)+ 1) - 1):0] flash_command_rx_idx;
reg [($clog2($bits(flash_commands)+ 1) - 1):0] flash_command_do_idx;
reg [7:0] flash_command_written_data;

// CART_WRITE_FLASH_CMD working registers
reg [1:0]  fcmd_p_idx;
reg [7:0]  fcmd_entry_count; // number of entries remaining
flash_command_t [0:FLASH_COMMANDS_MAX - 1] fcmd_par;
reg [2:0]  fcmd_idx;
reg [15:0] fcmd_address;
reg [2:0] fcmd_par_idx;

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
reg [1:0]  crc_p_remaining;
reg [15:0] crc_remaining;
reg [31:0] crc_state;
reg [15:0] crc_address;

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

reg vars_cleaned = 0;
always @(posedge clk) begin
    if (reset || !lk_enabled) begin
        vars_reset <= !vars_cleaned;
        vars_cleaned <= 1;
    end else vars_cleaned <= 0;
end

wire cmd_query_fw_info_complete;
wire cmd_query_fw_info_tx_valid;
wire [7:0] cmd_query_fw_info_tx_data;

lk_cmd_query_fw_info_t cmd_query_fw_info(
    .clk(clk),
    .en(command == CMD_QUERY_FW_INFO),
    .complete(cmd_query_fw_info_complete),
    .rx_valid(rx_valid),
    .rx_data(rx_data),
    .tx_valid(cmd_query_fw_info_tx_valid),
    .tx_data(cmd_query_fw_info_tx_data)
);

wire cmd_set_pin_complete;

lk_cmd_set_pin_t lk_cmd_set_pin(
    .clk(clk),
    .en(command == CMD_SET_PIN),
    .complete(cmd_set_pin_complete),
    .rx_valid(rx_valid),
    .rx_data(rx_data),
    .cart_audio(cart_audio)
);

wire cmd_dmg_mbc_reset_complete;
wire [15:0] cmd_dmg_mbc_reset_cart_a;
wire [7:0] cmd_dmg_mbc_reset_cart_d_out;
wire cmd_dmg_mbc_reset_cart_req;

lk_cmd_dmg_mbc_reset_t lk_dmg_cmd_reset(
    .clk(clk),
    .en(command == CMD_DMG_MBC_RESET),
    .complete(cmd_dmg_mbc_reset_complete),
    .cart_a(cmd_dmg_mbc_reset_cart_a),
    .cart_d_out(cmd_dmg_mbc_reset_cart_d_out),
    .cart_req(cmd_dmg_mbc_reset_cart_req),
    .cart_complete(cart_complete)
);

reg cmd_complete;
always_comb begin
    cmd_complete = 1; // default to going back to CMD_IDLE
    if (lk_enabled && !reset) begin
        case (command)
            CMD_IDLE: cmd_complete = rx_valid;
            CMD_GET_VARIABLE,
            CMD_SET_VARIABLE: cmd_complete = vars_cmd_complete;
            CMD_QUERY_FW_INFO: cmd_complete = cmd_query_fw_info_complete;
            CMD_SET_PIN: cmd_complete = cmd_set_pin_complete;
            CMD_DMG_MBC_RESET: cmd_complete = cmd_dmg_mbc_reset_complete;
            default: ;
        endcase
    end
end

command_t next_command;
always_comb begin
    next_command = CMD_IDLE;
    if (lk_enabled && !reset) begin
        if (command == CMD_IDLE && rx_valid) begin
            next_command = rx_data;
        end else next_command = cmd_complete ? CMD_IDLE : command;
    end
end

always_comb begin
    tx_valid = 1'b0;
    tx_data = 8'd0; // 8'h55, 8'd85, ascii uppercase U

    case (command)
        CMD_QUERY_FW_INFO: begin
            tx_valid = cmd_query_fw_info_tx_valid;
            tx_data = cmd_query_fw_info_tx_data;
        end
        CMD_GET_VARIABLE,
        CMD_SET_VARIABLE: begin
            tx_valid = vars_tx_valid;
            tx_data = vars_tx_data;
        end
        // Completion acks
        CMD_DMG_MBC_RESET,
        CMD_SET_PIN: begin
            tx_valid = cmd_complete;
            tx_data = 8'd1;
        end
        // Always-acks
        //
        // These are mostly commands that are unneeded for devices that only support DMG
        // but not AGB.
        //
        // FlashGBX invokes `DISABLE_PULLUPS` even for DMG, however, `ENABLE_PULLUPS` depends on
        // the cartridge type, and as of 2026-05-16, every cartridge that requires it is an AGB
        // cartridge.
        //
        // So for now, we need to ack `DISABLE_PULLUPS`, and we don't need to handle `ENABLE_PULLUPS` as
        // it should never be invoked
        CMD_DISABLE_PULLUPS,
        CMD_SET_MODE_DMG,
        CMD_SET_VOLTAGE_5V: begin
            tx_valid = 1;
            tx_data = 8'd1;
        end
        default: ;
    endcase
end

struct packed {
    reg valid;
    reg data_dir_e;
    reg a;
    reg d_out;
} cart_req;

always_comb begin
    cart_req = '{default:0};

    case (command)
        CMD_DMG_MBC_RESET: begin
            cart_req = '{
                valid: cmd_dmg_mbc_reset_cart_req,
                data_dir_e: 1'b0, // always a write
                a: cmd_dmg_mbc_reset_cart_a,
                d_out: cmd_dmg_mbc_reset_cart_d_out
            };
        end
        default: ;
    endcase
end

// ============================================================
// Main state machine
// ============================================================

always @(posedge clk) begin
    // Pulses
    command <= next_command;
    if (reset || !lk_enabled) begin
        cart_a          <= 16'hFFFF;
        cart_data_dir_e <= 1'b1; // read
        cart_state      <= C_IDLE;
        cart_clk        <= 1'b1;
        cart_cs         <= 1'b1;
        cart_rd         <= 1'b1;
        cart_wr         <= 1'b1;
        cart_rst_out    <= 1'b1;
    end else begin
        if (cart_req.valid) begin
            cart_data_dir_e <= cart_req.data_dir_e;
            cart_a <= cart_req.a;
            cart_d_out <= cart_req.d_out;
            cart_state <= C_SETUP;
        end
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
                if (cart_data_dir_e == 1'b0) begin
                    cart_cs       <= ~var_dmg_write_cs_pulse;
                    cart_wait_cnt <= CART_WR_HOLD[4:0] - 5'd1;
                    cart_state    <= C_WR_LOW;
                end else begin
                    cart_cs       <= ~var_dmg_read_cs_pulse;
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
                cart_state      <= C_DONE;
            end
        end

        C_DONE: begin
            cart_done  <= 1'b1;
            cart_state <= C_IDLE;
        end
        endcase // cart_state

`ifdef NOT_DEFINED
        case (command)
            CMD_IDLE: if (rx_valid) command <= command_t'{rx_data};
            CMD_BYE: begin
                case (bye_state)
                    BYE_RX: begin
                        if (rx_valid) begin
                            tx_valid <= 1;
                            // Got K, is this a 'KL'?
                            if (rx_data == "L") begin
                                tx_data_sel <= TXS_CONSTANT_FF;
                                bye_state <= BYE_EXEC;
                            end else begin
                                tx_data_sel <= TXS_CONSTANT_ZERO;
                                reset_bye();
                                command <= CMD_IDLE;
                            end
                        end
                    end // BYE_RX
                    BYE_EXEC: begin
                        lk_disable <= 1;
                        reset_bye();
                        command <= CMD_IDLE;
                    end // BYE_EXEC
                    default: ;
                endcase
            end // CMD_BYE

            CMD_QUERY_FW_INFO: begin
                tx_valid <= 1;
                tx_data_sel <= TXS_QUERY_FW_INFO;
                tx_pos <= tx_pos + 1;
                if (tx_pos == FWI_LEN[4:0] - 5'd1) begin
                    command <= CMD_IDLE;
                end
            end // CMD_QUERY_FW_INFO

            default: begin
                tx_data_sel <= TXS_CONSTANT_FF;
                tx_valid <= 1;
                command <= CMD_IDLE;
            end // default
        endcase

        P_CMD: begin
            if (rx_valid) begin
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
                    cart_state      <= C_SETUP;
                    pstate          <= P_CART_WR_DO;
                end

                8'hD5: begin // CALC_CRC32
                    crc_address <= var_address;
                    crc_p_remaining <= 2'd3;
                    crc_state <= ~32'd0;

                    pstate <= P_CALC_CRC_P;
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
                    pstate <= P_GET_VAR_STATE;
                end

                8'hAF: begin // SET_VAR_STATE: receive VSTATE_LEN bytes (ignored)
                    pstate <= P_SET_VAR_STATE_INIT;
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
                    xfer_remain <= var_transfer_size;
                    blob_idx    <= 0;
                    pstate      <= P_SRAM_WR_RX;
                end

                8'hD3: begin // FLASH_PROGRAM: receive XFER_SIZE bytes, write each
                    xfer_remain <= var_transfer_size;
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

        P_BYE_WAIT_L: begin
            if (rx_valid) begin
                pstate <= P_CMD;
                if (rx_data == "L") begin
                    tx_data_sel <= TXS_CONSTANT_FF;
                    tx_valid <= 1'b1;
                    lk_disable <= 1'b1;
                end
            end
        end // P_BYE_WAIT_L

        P_CALC_CRC_P: begin
            // We have var_address already set via SET_FW_VARIABLE
            // Now we need to get the 4-byte BE chunk length
            if (rx_valid) begin
                // we'll end up discarding the 2 MSB, but they're only for AGB,
                // and always 0 on DMG
                crc_remaining <= { crc_remaining[7:0], rx_data };
                if (crc_p_remaining == 0) pstate <= P_CALC_CRC_RD;
                crc_p_remaining <= crc_p_remaining - 2'd1;
            end
        end

        P_CALC_CRC_RD: begin
            cart_a <= crc_address;
            cart_data_dir_e <= 1'b1;
            cart_state <= C_SETUP;
            pstate <= P_CALC_CRC;
        end

        P_CALC_CRC: begin
            if (cart_done) begin
                if (crc_remaining == 16'd1) begin
                    // Like most LK commands, FlashGBX does the MB <-> LE conversion
                    crc_state <= next_crc(crc_state, cart_din_r) ^ 32'hFFFFFFFF;

                    tx_data_sel    <= TXS_CALC_CRC;
                    tx_bytes_count <= 4;
                    pstate         <= P_TX_BYTES;
                end else begin
                    crc_state <= next_crc(crc_state, cart_din_r);
                    crc_remaining <= crc_remaining - 16'd1;
                    crc_address <= crc_address + 16'd1;
                    pstate <= P_CALC_CRC_RD;
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
            // var_flash_we_pin`; however we don't support 4 == WR_RESET, which conflicts
            // with 'DEFAULT' for the override register. However, mapping 'unsupported'
            // to default seems reasonable for now.
            if (rx_valid) begin
                if (flash_command_rx_idx == 2) begin
                    var_flash_we_pin        <= rx_data;
                    flash_command_count  <= 0;
                    flash_command_rx_idx <= 0;
                    pstate <= P_SET_FLASH_CMD_E;
                end else begin
                    flash_command_rx_idx <= flash_command_rx_idx + 1;
                end
            end
        end

        P_SET_FLASH_CMD_E: begin
            if (rx_valid) begin
                // RX:   4 bytes for address, 2 for data
                // Used: 2 bytes for address, 1 for data
                case (flash_command_rx_idx)
                    3'd0, 3'd1: /* AGB only */ ;
                    3'd2: flash_command_address[15:8] <= rx_data;
                    3'd3: flash_command_address[7:0] <= rx_data;
                    3'd4: /* AGB only */ ;
                    3'd5: begin
                        flash_commands[flash_command_count].address <= flash_command_address;
                        flash_commands[flash_command_count].data <= rx_data;

                        if (flash_command_count == 0) pstate <= P_SET_FLASH_CMD_UPDATE_COUNT;

                        flash_command_count <= flash_command_count + 1;
                    end
                    default: /* unreachable */;
                endcase
                flash_command_rx_idx <= flash_command_rx_idx + 1;
            end
        end

        P_SET_FLASH_CMD_UPDATE_COUNT: begin
            // We enter this with flash_command_count == FLASH_COMMANDS_MAX
            for (integer i = 0; i < FLASH_COMMANDS_MAX; i = i + 1) begin
                if (flash_commands[i].address == 16'h0000 && flash_commands[i].data == 8'h00) begin
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
                tx_bytes_count <= tx_bytes_count - 1'b1;
                if (tx_bytes_count == 0) begin
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
        P_SET_VAR_INIT: begin
            set_var_p_idx <= 0;
            pstate <= P_SET_VAR_P;
        end
        P_SET_VAR_P: begin
            if (rx_valid) begin
                case (set_var_p_idx)
                    4'd0: set_var_size <= rx_data[2:0];
                    4'd1, 4'd2, 4'd3: /* unused key MSB */ ;
                    4'd4: set_var_key <= rx_data[4:0];
                    4'd5, 4'd6: /* unused data MSB */ ;
                    4'd7:
                        set_var_data[15:8] <= rx_data;
                    4'd8: begin
                        set_var_data[7:0] <= rx_data;
                        set_var_rdy <= 1'b1;
                        pstate <= P_TX_ACK;
                    end
                endcase
                set_var_p_idx <= set_var_p_idx + 1;
            end
        end
        // ── GET_VARIABLE: read size(1)+key(4) = 5 bytes ─────────────────
        P_GET_VAR_INIT: begin
            get_var_p_idx <= 0;
            pstate <= P_GET_VAR_P;
        end
        P_GET_VAR_P: begin
            if (rx_valid) begin
                case (get_var_p_idx)
                    3'd0: get_var_size <= rx_data[2:0];
                    3'd1, 3'd2, 3'd3: /* unused key MSB */ ;
                    3'd4: begin
                        get_var_data    <= get_var32(get_var_size, rx_data[4:0]);
                        tx_data_sel     <= TXS_GET_VAR;
                        tx_bytes_count  <= 4;
                        pstate          <= P_TX_BYTES;
                    end
                endcase
                get_var_p_idx <= get_var_p_idx + 1;
            end
        end

        // ── GET_VAR_STATE: dump all variables ──────────────────────────
        // This is used to save/restore state around a power cycle or USB replug, and
        // the format is opaque to the protocol peer.
        //
        // So, order/encoding doesn't matter, just needs to be consistent between
        // GET_VAR_STATE and SET_VAR_STATE
        P_GET_VAR_STATE: begin
            tx_data_sel <= TXS_GET_VAR_STATE;
            tx_bytes_count <= VARS_BYTE_COUNT;
            pstate <= P_TX_BYTES;
        end

        P_SET_VAR_STATE_INIT: begin
            set_var_state_remaining <= VARS_BYTE_COUNT;
            pstate <= P_SET_VAR_STATE;
        end

        P_SET_VAR_STATE: begin
            if (rx_valid) begin
                set_var_state_data <= rx_data;
                set_var_state_rdy <= 1;
                set_var_state_idx <= VARS_BYTE_COUNT - set_var_state_remaining;

                if (set_var_state_remaining == 1) pstate <= P_CMD;
                set_var_state_remaining <= set_var_state_remaining - 1;
            end
        end

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
            xfer_remain <= var_transfer_size;
            cart_a          <= var_address;
            cart_data_dir_e <= 1'b1;
            cart_state      <= C_SETUP;
            pstate          <= P_CART_RD_TX;
        end


        P_CART_RD_TX: begin
            // SRAM path: wait for cart access to complete
            if (cart_done) begin
                tx_data_sel <= TXS_CART_IN;
                tx_valid <= 1'b1;
                var_address    <= var_address + 32'd1;
                xfer_remain <= xfer_remain - 16'd1;
                if (xfer_remain == 16'd1) begin
                    pstate <= P_CMD;
                end else begin
                    // Kick off next byte
                    cart_a          <= var_address + 16'd1;
                    cart_data_dir_e <= 1'b1;
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
                        dmg_cart_write_a <= { dmg_cart_write_a[7:0], rx_data };
                    end
                    3'd4: begin
                        cart_a          <= dmg_cart_write_a;
                        cart_d_out      <= rx_data;
                        cart_data_dir_e <= 1'b0;
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
                if (blob_idx == var_transfer_size - 1) begin
                    blob_idx <= 0;
                    pstate   <= P_SRAM_WR_DO;
                end else begin
                    blob_idx <= blob_idx + 1;
                end
            end
        end

        P_SRAM_WR_DO: begin
                cart_a          <= var_address;
                cart_d_out      <= blob[blob_idx];
                cart_data_dir_e <= 1'b0;
                cart_state      <= C_SETUP;
                pstate          <= P_SRAM_WR_WAIT;
        end

        P_SRAM_WR_WAIT: begin
            if (cart_done) begin
                var_address <= var_address + 16'd1;
                if (blob_idx == var_transfer_size - 1) begin
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
            flash_commands[flash_command_count].address <= var_address + blob_idx;
            flash_commands[flash_command_count].data <= blob[blob_idx];
            flash_command_written_data <= blob[blob_idx];

            flash_command_do_idx <= 0;
            pstate <= P_FLASH_PROGRAM_WR_DO;
        end

        P_FLASH_PROGRAM_WR_DO: begin
            cart_write_pulse_pins <= var_flash_we_pin ? CART_WRITE_PULSE_PINS_WR : CART_WRITE_PULSE_PINS_NONE;
            cart_a <= flash_commands[flash_command_do_idx].address;
            cart_d_out <= flash_commands[flash_command_do_idx].data;
            cart_data_dir_e <= 1'b0;
            cart_state <= C_SETUP;

            pstate <= P_FLASH_PROGRAM_WR_WAIT_WRITE;
        end

        P_FLASH_PROGRAM_WR_WAIT_WRITE: begin
            if (cart_done) begin
                if (flash_command_do_idx == flash_command_count) begin
                    // After the queued commands, we have an extra one that's the actual data
                    cart_data_dir_e <= 1'b1;
                    cart_state <= C_SETUP;
                    pstate <= P_FLASH_PROGRAM_WR_WAIT_STATUS;
                end else begin
                    flash_command_do_idx <= flash_command_do_idx + 1;
                    pstate <= P_FLASH_PROGRAM_WR_DO;
                end
            end
        end

        P_FLASH_PROGRAM_WR_WAIT_STATUS: begin
            if (cart_done) begin
                var_status_register <= cart_d_in;
                if (cart_d_in[7] == flash_command_written_data[7]) begin
                    if (blob_idx == var_transfer_size - 16'd1) begin
                        cart_write_pulse_pins <= CART_WRITE_PULSE_PINS_DEFAULT;
                        pstate <= P_TX_ACK;
                        // LK_Device::WriteROM only sets ADDRESS on the first chunk,
                        // so we need to increment it before the next one.
                        var_address <= var_address + var_transfer_size;
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
                        flb_wr_a <= { flb_wr_a[7:0], rx_data};
                    end
                    3'd4: begin
                        cart_write_pulse_pins <= var_flash_we_pin ? CART_WRITE_PULSE_PINS_WR : CART_WRITE_PULSE_PINS_NONE;
                        cart_a          <= flb_wr_a;
                        cart_d_out      <= rx_data;
                        cart_data_dir_e <= 1'b0;
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
                        fcmd_par_idx <= 0;
                        if (rx_data == 0) begin
                            pstate <= P_TX_ACK;
                        end else begin
                            fcmd_idx <= 3'd0;
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
                fcmd_idx <= fcmd_idx + 8'd1;

                case (fcmd_idx)
                    3'd0, 3'd1: /* MSB for AGB only */ ;
                    3'd2: fcmd_address[15:8] <= rx_data;
                    3'd3: fcmd_address[7:0] <= rx_data;
                    3'd4: /* AGB only */ ;
                    3'd5: begin
                        fcmd_par[fcmd_par_idx].address <= fcmd_address;
                        fcmd_par[fcmd_par_idx].data <= rx_data;
                        if (fcmd_par_idx == fcmd_entry_count - 1) begin
                            fcmd_par_idx <= 0;
                            pstate <= P_CART_WRITE_FLASH_CMD_W_NOWAIT;
                        end else begin
                            fcmd_par_idx <= fcmd_par_idx + 1;
                            fcmd_idx <= 0;
                        end
                    end
                endcase
            end
        end

        P_CART_WRITE_FLASH_CMD_W, P_CART_WRITE_FLASH_CMD_W_NOWAIT: begin
            if (cart_done || pstate == P_CART_WRITE_FLASH_CMD_W_NOWAIT) begin
                if (fcmd_entry_count == 8'd0) begin
                    cart_write_pulse_pins <= CART_WRITE_PULSE_PINS_DEFAULT;
                    cart_data_dir_e <= 1'b1;
                    cart_state <= C_SETUP;
                    pstate <= P_CART_WRITE_FLASH_CMD_WAIT_STATUS;
                end else begin
                    // Entry complete
                    cart_a <= fcmd_par[fcmd_par_idx].address;
                    cart_d_out <= fcmd_par[fcmd_par_idx].data;
                    cart_data_dir_e <= 1'b0;
                    cart_write_pulse_pins <= var_flash_we_pin ? CART_WRITE_PULSE_PINS_WR : CART_WRITE_PULSE_PINS_NONE;
                    cart_state <= C_SETUP;

                    fcmd_entry_count <= fcmd_entry_count - 8'd1;
                    fcmd_par_idx <= fcmd_par_idx + 1;
                    pstate <= P_CART_WRITE_FLASH_CMD_W;
                end
            end
        end

        P_CART_WRITE_FLASH_CMD_WAIT_STATUS: begin
            if (cart_done) begin
                var_status_register <= cart_d_in;
                pstate <= P_TX_ACK;
            end
        end

        default: pstate <= P_CMD;
        endcase // pstate
`endif // ifdef old paste

    end // ~reset
end // always

endmodule // cart_reader
`default_nettype wire
