import lk_types::*;

module lk_top(
    input  wire        usbClk,
    input  wire        cartClk,
    input  wire        reset,

    // USB-CDC
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    output reg         tx_valid,
    output reg  [7:0]  tx_data,

    output reg         cart_enabled,

    output reg  [15:0] cart_a,
    output reg         cart_clk,
    output reg         cart_cs,
    output reg         cart_rd,
    output reg         cart_wr,
    output reg         cart_data_dir_e,   // 1 = read, 0 = write
    output reg  [7:0]  cart_d_out,        // data to write
    input  wire [7:0]  cart_d_in,         // data read from cart
    output tristate_pin_t cart_rst,
    output tristate_pin_t cart_audio
);
assign cart_enabled = 1'b1;

(* syn_preserve *) reg [1:0] lk_cdc_usb_reset;
always @(posedge usbClk or posedge reset) begin
    if (reset) lk_cdc_usb_reset <= 2'b11;
    else       lk_cdc_usb_reset <= { lk_cdc_usb_reset[0], 1'b0 };
end
wire usbReset = lk_cdc_usb_reset[1];

(* syn_preserve *) reg [1:0] lk_cdc_cart_reset;
always @(posedge cartClk or posedge reset) begin
    if (reset) lk_cdc_cart_reset <= 2'b11;
    else       lk_cdc_cart_reset <= { lk_cdc_cart_reset[0], 1'b0 };
end
wire cartReset = lk_cdc_cart_reset[1];

////// FIFO /////
typedef logic[31:0] fifo_req_t;

logic req_enqueue;
fifo_req_t req_enqueue_data;
logic req_dequeue;
logic reqs_empty;
fifo_req_t req_q;

lk_cart_fifo_t u_request_fifo(
    .WrClk(usbClk),
    .WrEn(req_enqueue),
    .Data(req_enqueue_data),

    .RdClk(cartClk),
    .RdEn(req_dequeue),
    .Q(req_q),
    .Empty(reqs_empty),

    .Almost_Full(),
    .Full()
);

typedef logic[31:0] fifo_response_t;
logic response_enqueue;
fifo_response_t response_enqueue_data;
logic response_dequeue;
logic responses_empty;
fifo_response_t response_q;

lk_cart_fifo_t u_responseuest_fifo(
    .WrClk(cartClk),
    .WrEn(response_enqueue),
    .Data(response_enqueue_data),

    .RdClk(usbClk),
    .RdEn(response_dequeue),
    .Q(response_q),
    .Empty(responses_empty),

    .Almost_Full(),
    .Full()
);

///// END FIFO /////

typedef enum {
  S_RESET,
  S_IDLE,
  S_EXEC
} state_t;

typedef enum logic [3:0] {
  CMD_PING = 4'd0,
  CMD_DELAY = 4'd1,
  CMD_SET_PINS = 4'd2,
  CMD_SET_DIRECTION = 4'd3,
  CMD_SET_ADDRESS = 4'd4,
  CMD_SET_DATA = 4'd5,
  CMD_GET_DATA = 4'd6,

  CMD_INIT_ACK = 4'd7, // not a real command
  CMD_IDLE = 4'd8 // not a real command
} command_t;
localparam command_t CMD_COUNT = CMD_IDLE;
logic [CMD_COUNT - 1:0] complete_bus;
wire command_complete = |complete_bus;

state_t state;
state_t state_next;

command_t command;
command_t command_next;

always @(*) begin
    state_next = state;
    command_next = command;
    req_dequeue = 1'b0;

    unique case (state)
        S_RESET: begin
            state_next = S_EXEC;
            command_next = CMD_INIT_ACK;
        end
        S_IDLE: if (!reqs_empty) begin
            req_dequeue = 1'b1;
            state_next = S_EXEC;
            // Only look at 3 bits because CMD_IDLE and CMD_INIT_ACK can not be explicitly selected
            command_next = command_t'(req_q[26:24]);
        end
        S_EXEC: begin
            if (command_complete) begin
                state_next = S_IDLE;
                command_next = CMD_IDLE;
            end
        end
        default: ;
    endcase
end

always @(posedge cartClk) begin
    if (cartReset) begin
        state <= S_RESET;
        command <= CMD_IDLE;
    end else begin
        state <= state_next;
        command <= command_next;
    end
end

logic [15:0] arg16;
logic [7:0] arg8a;
logic [7:0] arg8b;
assign arg8a = arg16[15:8];
assign arg8b = arg16[7:0];
always @(posedge cartClk) begin
    if (req_dequeue) begin
        arg16 <= req_q[23:8];
    end
end

assign complete_bus[CMD_INIT_ACK] = (command == CMD_INIT_ACK);
assign complete_bus[CMD_PING] = (command == CMD_PING);

typedef enum {
    DELAY_IDLE,
    DELAY_WAIT,
    DELAY_COMPLETE
} delay_state_t;
delay_state_t delay_state;
logic [21:0] delay_ticks;


always @(posedge cartClk) begin
    delay_ticks <= delay_ticks;
    if (cartReset) begin
        delay_state <= DELAY_IDLE;
    end else if (command == CMD_DELAY) begin
        delay_state <= delay_state;
        unique case (delay_state)
            DELAY_IDLE: begin
                // Approximate microseconds to 59.605ns clock ticks
                // We want a multiplicand of (1000 / 59.605), which is ~= 16.777
                // Instead, we go for 17 - (1/4) + (1/32), done with shifts, which is 16.78125, and easy
                // to do with shifts
                //
                // There should be a '+1', but given we have the delay_wait/delay_complete states,
                // we're a small constant number of ticks too high anyway :)
                delay_ticks <= (arg16 << 4) + arg16 - (arg16 >> 2) + (arg16 >> 5);
                delay_state <= DELAY_WAIT;
            end
            DELAY_WAIT: begin
                if (delay_ticks > 0) begin
                    delay_ticks <= delay_ticks - 1'd1;
                end else begin
                    delay_state <= DELAY_COMPLETE;
                end
            end
            default: ;
        endcase
    end else begin
        delay_state <= DELAY_IDLE;
    end
end
assign complete_bus[CMD_DELAY] = (delay_state == DELAY_COMPLETE);

always @(posedge cartClk) begin
    response_enqueue <= 1'b0;
    response_enqueue_data <= '{default: 0};
    if ((state == S_EXEC) && command_complete) begin
        response_enqueue <= 1'b1;
        response_enqueue_data[27:24] <= command;
        unique case (command)
            CMD_PING: begin
                response_enqueue_data[23:16] <= ~arg8a;
            end
            default: begin
                response_enqueue_data[16] <= 1'b1;
            end
        endcase
    end
end

endmodule