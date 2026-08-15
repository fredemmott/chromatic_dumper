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

///// BEGIN USB <-> FIFOS /////
logic [1:0] rx_count;
always @(posedge usbClk) begin
    req_enqueue <= 1'b0;
    if (usbReset) begin
        rx_count <= 0;
        req_enqueue_data <= '{default: 0};
    end else if (rx_valid) begin
        rx_count <= rx_count + 1'd1;
        unique case (rx_count)
            0: req_enqueue_data[31:24] <= rx_data;
            1: req_enqueue_data[23:16] <= rx_data;
            2: begin
                req_enqueue_data[15:8] <= rx_data;
                req_enqueue <= 1'b1;
                rx_count <= 2'd0;
            end
        endcase
    end
end


logic tx_count;
assign response_dequeue = (!responses_empty) && (tx_count == 1);

always @(posedge usbClk) begin
    tx_valid <= 1'b0;
    if (usbReset) begin
        tx_count <= 1'd0;
    end else if (!responses_empty) begin
        tx_valid <= 1'b1;
        if (tx_count == 1'd0) begin
            tx_data <= response_q[31:24];
        end else begin
            tx_data <= response_q[23:16];
        end
        tx_count <= ~tx_count;
    end
end

///// END USB <-> FIFOS /////

typedef enum {
  S_RESET,
  S_IDLE,
  S_EXEC
} state_t;

state_t state;
state_t state_next;

command_t command;
command_t command_next;

logic delay_complete;
logic command_is_delay;
assign command_complete = (state == S_EXEC) && ((!command_is_delay) || delay_complete);

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

typedef enum {
    DELAY_IDLE,
    DELAY_WAIT,
    DELAY_COMPLETE
} delay_state_t;
delay_state_t delay_state;
logic [21:0] delay_ticks;
assign delay_complete = (delay_state == DELAY_COMPLETE);
assign command_is_delay = (command == CMD_DELAY_MICROS) || (command == CMD_DELAY_NANOS);

always @(posedge cartClk) begin
    delay_ticks <= delay_ticks;
    if (cartReset) begin
        delay_state <= DELAY_IDLE;
    end else if (command_is_delay) begin
        delay_state <= delay_state;
        unique case (delay_state)
            DELAY_IDLE: begin
                delay_state <= DELAY_WAIT;
                if (command == CMD_DELAY_MICROS) begin
                    // Approximate microseconds to 59.605ns clock ticks
                    // We want a multiplicand of (1000 / 59.605), which is ~= 16.777
                    // Instead, we go for 17 - (1/4) + (1/32), done with shifts, which is 16.78125, and easy
                    // to do with shifts
                    //
                    // There should be a '+1', but given we have the delay_wait/delay_complete states,
                    // we're a small constant number of ticks too high anyway :)
                    delay_ticks <= (arg16 << 4) + arg16 - (arg16 >> 2) + (arg16 >> 5);
                end else if (command == CMD_DELAY_NANOS) begin
                    delay_ticks <= (arg16 >> 6) + (arg16 >> 10) + (arg16 >> 13) + (arg16 >> 14);
                end
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

always @(posedge cartClk) begin
    response_enqueue <= 1'b0;
    response_enqueue_data <= '{default: 0};
    if ((state == S_EXEC) && command_complete) begin
        response_enqueue <= 1'b1;
        response_enqueue_data[27:24] <= 4'(command);
        unique case (command)
            CMD_PING: begin
                response_enqueue_data[23:16] <= ~arg8a;
            end
            CMD_INIT_ACK: begin
                response_enqueue_data[31:16] <= ~"LK";
            end
            CMD_GET_DATA: begin
                response_enqueue_data[23:16] <= cart_d_in;
            end
            default: begin
                response_enqueue_data[16] <= 1'b1;
            end
        endcase
    end
end

`define SET_PIN(TARGET, IDX) \
        if (arg8a[IDX]) TARGET <= arg8b[IDX];
`define SET_TRISTATE_PIN(TARGET, IDX) \
        if (arg8a[IDX]) begin \
            TARGET.oe <= 1'b1; \
            TARGET.value <= arg8b[IDX]; \
        end

        // A15 is handled in SET_ADDRESS
always @(posedge cartClk) begin
    if (cartReset) begin
        cart_clk <= 1'b1;
        cart_wr <= 1'b1;
        cart_rd <= 1'b1;
        cart_cs <= 1'b1;
        cart_rst <= '{default: 0};
        cart_audio <= '{default: 0};
    end else if (command == CMD_SET_PINS) begin
        `SET_PIN(cart_clk, SET_PIN_CLK);
        `SET_PIN(cart_wr, SET_PIN_WR);
        `SET_PIN(cart_rd, SET_PIN_RD)
        `SET_PIN(cart_cs, SET_PIN_CS)
        `SET_TRISTATE_PIN(cart_rst, SET_PIN_RST);
        `SET_TRISTATE_PIN(cart_audio, SET_PIN_AUDIO);
    end else if (command == CMD_SET_OUTPUT_ENABLE) begin
        if (arg8a[OE_AUDIO]) begin
            cart_audio.oe <= arg8b[OE_AUDIO];
        end
    end
end

always @(posedge cartClk) begin
    if (cartReset) begin
        cart_data_dir_e <= 1'b1; // read
    end else if ((command == CMD_SET_OUTPUT_ENABLE) && arg8a[OE_DATA]) begin
        cart_data_dir_e <= ~arg8b[OE_DATA];
    end
end

always @(posedge cartClk) begin
    if (cartReset) begin
        cart_a <= '{default: 0};
    end else if (command == CMD_SET_ADDRESS) begin
        cart_a <= arg16;
    end else if ((command == CMD_SET_PINS) && arg8b[SET_PIN_A15]) begin
        cart_a[15] <= arg8b[SET_PIN_A15];
    end
end

always @(posedge cartClk) begin
    if (cartReset) begin
        cart_d_out <= '{default: 0};
    end else if (command == CMD_SET_DATA) begin
        cart_d_out <= arg8a;
    end
end

endmodule