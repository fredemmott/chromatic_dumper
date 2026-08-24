package lk_types;

typedef enum logic [7:0] {
  CMD_NOP = 8'd0,
  CMD_PING = 8'd1,

  CMD_SET_PINS = 8'd2,
  CMD_SET_OUTPUT_ENABLE = 8'd3,
  CMD_SET_ADDRESS = 8'd4,
  CMD_SET_DATA = 8'd5,
  CMD_GET_DATA = 8'd6
} command_t;


typedef struct {
    logic oe;
    logic value;
} tristate_pin_t;

localparam SET_PIN_CLK = 0;
localparam SET_PIN_WR = 1;
localparam SET_PIN_RD = 2;
localparam SET_PIN_CS = 3;
localparam SET_PIN_A15 = 4;
localparam SET_PIN_RST = 5; // a.k.a. 'CS2' (AGB)
localparam SET_PIN_AUDIO = 6;

localparam OE_AUDIO = 0;
localparam OE_DATA = 1;

endpackage