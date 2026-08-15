package lk_types;

typedef enum logic [3:0] {
  CMD_PING = 4'd0,
  CMD_DELAY_MICROS = 4'd1,
  CMD_DELAY_NANOS = 4'd2,
  CMD_SET_PINS = 4'd3,
  CMD_SET_OUTPUT_ENABLE = 4'd4,
  CMD_SET_ADDRESS = 4'd5,
  CMD_SET_DATA = 4'd6,
  CMD_GET_DATA = 4'd7,

  CMD_INIT_ACK = 4'd8, // not a real command
  CMD_IDLE = 4'd9 // not a real command
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