package lk_types;

typedef enum {
  CMD_PING,
  CMD_DELAY,
  CMD_SET_PINS,
  CMD_SET_DIRECTION,
  CMD_SET_ADDRESS,
  CMD_SET_DATA,
  CMD_GET_DATA
} command_t;

typedef struct {
    logic oe;
    logic value;
} tristate_pin_t;

endpackage