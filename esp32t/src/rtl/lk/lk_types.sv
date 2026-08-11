package lk_types;
localparam FLASH_WE_PIN_WR = 2'd1;
localparam FLASH_WE_PIN_AUDIO  = 2'd2;
localparam FLASH_WE_PIN_WR_AND_RESET = 2'd3;

typedef struct {
    logic hold_pin_audio;
    logic [1:0] flash_we_pin;
    logic dmg_read_cs_pulse;
    logic dmg_write_cs_pulse;
} cart_vars_t;

typedef struct {
    logic        is_flash;
    logic        is_write;
    logic        wait_for_status;
    logic [15:0] address;
    logic [7:0]  data;
} cart_req_t;

endpackage