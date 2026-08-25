package lk_types;

typedef enum logic [7:0] {
    CMD_NOP = 8'd0,
    CMD_PING = 8'd1,

    CMD_SET_ADDRESS_MSB = 8'd2,
    CMD_SET_ADDRESS_LSB = 8'd3,
    CMD_SET_OUTPUT_ENABLE = 8'd4,
    CMD_SET_DATA = 8'd5,
    CMD_GET_DATA = 8'd6,
    CMD_SET_PINS_A = 8'd7,
    CMD_SET_PINS_B = 8'd8
} command_t;


typedef struct {
    logic oe;
    logic value;
} tristate_pin_t;

// indicies for bitmasks
localparam SET_PINS_A_CLK = 0;
localparam SET_PINS_A_WR = 1;
localparam SET_PINS_A_RD = 2;
localparam SET_PINS_A_CS = 3;

localparam SET_PINS_B_A15 = 0; // Also set by by SET_ADDRESS_MSB
localparam SET_PINS_B_RST = 1; // a.k.a. 'CS2' (AGB)
localparam SET_PINS_B_AUDIO = 2;

localparam OE_AUDIO = 0;
localparam OE_DATA = 1;

endpackage