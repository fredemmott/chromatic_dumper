package lk_types;

typedef enum {
    CMD_INVALID,
    CMD_STUB_NOOP_ACK,
    CMD_QUERY_FW_INFO,
    CMD_SET_VARIABLE,
    CMD_SET_VOLTAGE_5V,
    CMD_SET_ADDR_AS_INPUTS/*,
    CMD_SET_PIN/*,
    CMD_SET_FLASH_CMD,
    CMD_CLK_TOGGLE,
    CMD_GET_VARIABLE,
    CMD_GET_VAR_STATE,
    CMD_SET_VAR_STATE,
    CMD_DMG_CART_READ,
    CMD_DMG_CART_WRITE,
    CMD_DMG_CART_WRITE_SRAM,
    CMD_DMG_MBC_RESET,
    CMD_DMG_SET_BANK_CHANGE_CMD,
    CMD_DMG_FLASH_WRITE_BYTE,
    CMD_FLASH_PROGRAM,
    CMD_CART_WRITE_FLASH_CMD,
    CMD_CALC_CRC32,
    */
} command_t;
// TODO:
parameter command_t CMD_NOT_IMPLEMENTED = CMD_INVALID;

parameter command_t CMD_SET_FLASH_CMD = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_CLK_TOGGLE = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_GET_VARIABLE = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_GET_VAR_STATE = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_SET_VAR_STATE = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_DMG_CART_READ = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_DMG_CART_WRITE = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_DMG_CART_WRITE_SRAM = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_DMG_MBC_RESET = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_DMG_SET_BANK_CHANGE_CMD = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_DMG_FLASH_WRITE_BYTE = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_FLASH_PROGRAM = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_CART_WRITE_FLASH_CMD = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_CALC_CRC32 = CMD_NOT_IMPLEMENTED;
parameter command_t CMD_SET_PIN = CMD_NOT_IMPLEMENTED;

parameter command_t CMD_DISABLE_PULLUPS = CMD_STUB_NOOP_ACK;
parameter command_t CMD_SET_MODE_DMG = CMD_STUB_NOOP_ACK;

typedef struct packed {
    logic [15:0] address;
    logic [15:0] transfer_size;
    logic [7:0]  status_register;
    logic [7:0]  cart_mode;
    logic [7:0]  dmg_access_mode;
    logic [1:0]  flash_we_pin;
    logic        dmg_read_cs_pulse;
    logic        dmg_write_cs_pulse;
} vars_t;

typedef struct packed {
    logic valid;
    logic is_write;
    logic [15:0] address;
    logic [7:0] data;
} cart_out_t;

typedef struct packed {
    logic audio;
} cart_pins_t;

endpackage