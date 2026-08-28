package lk_types;

// All commands are two bytes: {command, arg}
//
// If a command doesn't take an argument, 'arg' is ignored
//
// - this allows creating a bunch of NOPs for delays with `memset()`
// - this gives us consistent timing - except for the VERIFY commands - which
//   is useful.
// - it makes state machines simpler
typedef enum logic [7:0] {
    CMD_NOP = 8'd0,
    CMD_PING = 8'd1, // ( cookie ) -> ~cookie

    CMD_SET_ADDRESS_MSB = 8'd2,
    CMD_SET_ADDRESS_LSB = 8'd3,
    CMD_SET_OUTPUT_ENABLE = 8'd4, // ({4 bits for OE pin select, 4 bits for values})
    CMD_SET_DATA = 8'd5,
    CMD_GET_DATA = 8'd6,
    CMD_SET_PINS_A = 8'd7, // ({4 bits for pin select, 4 bits for values})
    CMD_SET_PINS_B = 8'd8, // ({4 bits for pin select, 4 bits for values})
    CMD_VERIFY_DATA = 8'd9, // (expected byte)
    CMD_VERIFY_STATUS_REGISTER = 8'd10, // arg ignored
    CMD_SET_STATUS_REGISTER_MASK = 8'd11,
    CMD_SET_STATUS_REGISTER_VALUE = 8'd12
} command_t;

function command_produces_tx (command_t cmd);
    begin
        unique case(cmd)
            CMD_GET_DATA,
            CMD_PING,
            CMD_VERIFY_DATA: return 1'b1;
            CMD_VERIFY_STATUS_REGISTER: return 1'b1;
            default: return 1'b0;
        endcase
    end
endfunction

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
localparam OE_ADDRESS = 2;

endpackage