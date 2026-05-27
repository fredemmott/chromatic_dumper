package lk_types;
typedef enum {
    CMD_STUB_NOOP_ACK,
    CMD_QUERY_FW_INFO,
    CMD_SET_VARIABLE,
    CMD_SET_VOLTAGE_5V,
    CMD_SET_ADDR_AS_INPUTS,
    CMD_SET_PIN,
    CMD_DMG_MBC_RESET,
    CMD_GET_VARIABLE,
    CMD_DMG_CART_READ,
    CMD_DMG_CART_WRITE,
    CMD_CART_WRITE_FLASH_CMD,
    CMD_DMG_FLASH_WRITE_BYTE,
    /*
    CMD_SET_FLASH_CMD,
    CMD_CLK_TOGGLE,
    CMD_GET_VARIABLE,
    CMD_GET_VAR_STATE,
    CMD_SET_VAR_STATE,
    CMD_DMG_CART_WRITE_SRAM,
    CMD_DMG_SET_BANK_CHANGE_CMD,
    CMD_FLASH_PROGRAM,
    CMD_CALC_CRC32,
    */
    CMD_COUNT
} command_t;
parameter command_t CMD_INVALID = CMD_COUNT;
// TODO:
parameter command_t CMD_NOT_IMPLEMENTED = CMD_INVALID;

parameter command_t CMD_SET_FLASH_CMD = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_CLK_TOGGLE = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_GET_VAR_STATE = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_SET_VAR_STATE = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_DMG_CART_WRITE_SRAM = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_DMG_SET_BANK_CHANGE_CMD = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_FLASH_PROGRAM = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_CALC_CRC32 = CMD_NOT_IMPLEMENTED;

parameter command_t CMD_DISABLE_PULLUPS = CMD_STUB_NOOP_ACK;
parameter command_t CMD_SET_MODE_DMG = CMD_STUB_NOOP_ACK;

typedef struct {
    logic [15:0] address;
    logic [15:0] transfer_size;
    logic [7:0]  status_register;
    logic [1:0]  cart_mode;
    logic [2:0]  dmg_access_mode;
    logic [1:0]  flash_we_pin;
    logic        dmg_read_cs_pulse;
    logic        dmg_write_cs_pulse;
} vars_t;
localparam FLASH_WE_PIN_WR = 2'd1;
localparam FLASH_WE_PIN_AUDIO  = 2'd2;
localparam FLASH_WE_PIN_WR_AND_RESET = 2'd3;

// unused:
//
// localparam DMG_ACCESS_MODE_ROM_READ = 3'd1;
// localparam DMG_ACCESS_MODE_RAM_READ = 3'd3;
// localparam DMG_ACCESS_MODE_RAM_WRITE = 3'd4;
//
// localparam CART_MODE_DMG = 2'd1;
// localparam CART_MODE_AGB = 2'd2;

typedef struct {
    logic hold_pin_audio;
    logic [1:0] flash_we_pin;
    logic dmg_read_cs_pulse;
    logic dmg_write_cs_pulse;
} cart_vars_t;

typedef struct packed {
    logic        is_flash;
    logic        is_write;
    logic [15:0] address;
    logic [7:0]  data;
} cart_req_t;


endpackage