import lk_types::*;

module lk_top(
    input  wire        clk,
    input  wire        reset,

    output reg         rx_ready,
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    output reg         tx_valid,
    output reg  [7:0]  tx_data,

    output reg         cart_enabled,

    output reg  [15:0] cart_a,
    output reg         cart_a_oe,
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

logic [7:0] fifo [2047:0];
logic [10:0] fifo_read_p;
logic [10:0] fifo_write_p;
logic [11:0] fifo_count;
logic [7:0] fifo_q;
assign fifo_q = fifo[fifo_read_p];
assign rx_ready = (fifo_count <= 12'd1536); // 2048 - 512 (max packet size)

wire next_byte_valid = fifo_count > 12'd0;
wire [7:0] next_byte = fifo_q;
//wire next_byte_valid = rx_valid;
//wire [7:0] next_byte = rx_data;

typedef enum {
  S_IDLE,
  S_WAIT_ARG,
  S_EXEC_VERIFY // post-write CMD_VERIFY_DATA or CMD_VERIFY_STATUS_REGISTER
} state_t;
state_t state;
state_t state_d;
always @(posedge clk) begin
    state_d <= state;
end

logic next_byte_pop;
always @(*) begin
    next_byte_pop = 1'b0;
    if (next_byte_valid) begin
        unique case (state)
            S_IDLE, S_WAIT_ARG: next_byte_pop = 1'b1;
            default: ;
        endcase
    end
end

always @(posedge clk) begin
    if (reset) begin
        fifo_count <= 12'd0;
        fifo_read_p <= 11'd0;
        fifo_write_p <= 11'd0;
    end else begin
        if (rx_valid) begin
            fifo[fifo_write_p] <= rx_data;
            fifo_write_p <= fifo_write_p + 11'd1;
        end
        if (next_byte_pop) begin
            fifo_read_p <= fifo_read_p + 11'd1;
        end
        unique case ({rx_valid, next_byte_pop})
            2'b00, 2'b11: /* no change to count */ ;
            2'b01: fifo_count <= fifo_count - 12'd1;
            2'b10: fifo_count <= fifo_count + 12'd1;
        endcase
    end
end

typedef enum {
    VS_PIN_RD_L,
    VS_DELAY,
    VS_PIN_RD_H, // also read and TX here
    VS_COMPLETE
} verify_state_t;
verify_state_t verify_state;

command_t command;
command_t command_latched;
logic [7:0] arg;
logic [7:0] arg_latched;

always @(*) begin
    command = CMD_NOP;
    arg = 8'd0;
    unique case (state)
        S_IDLE: /* nop */;
        S_WAIT_ARG: begin
            // NOP *until* we receive the arg byte, then we have everything
            if (next_byte_valid) begin
                command = command_latched;
                arg = next_byte;
            end
        end
        default: begin
            command = command_latched;
            arg = arg_latched;
        end
    endcase
end


always @(posedge clk) begin
    state <= state;
    command_latched <= command_latched;
    arg_latched <= arg_latched;

    if (reset) begin
        state <= S_IDLE;
        command_latched <= CMD_NOP;
        arg_latched <= 8'd0;
    end else if (state == S_EXEC_VERIFY) begin
        if (verify_state == VS_COMPLETE) begin
            state <= S_IDLE;
        end
    end else if (next_byte_valid) begin
        unique case (state)
            S_IDLE: begin
                command_latched <= command_t'(next_byte);

                state <= S_WAIT_ARG;
            end
            S_WAIT_ARG: begin
                arg_latched <= next_byte;

                unique case (command)
                    CMD_VERIFY_DATA: state <= S_EXEC_VERIFY;
                    CMD_VERIFY_STATUS_REGISTER: state <= S_EXEC_VERIFY;
                    default: state <= S_IDLE;
                endcase
            end
            S_EXEC_VERIFY: /* nothing */ ;
            default: state <= S_IDLE;
        endcase
    end
end

logic [7:0] status_register_mask;
logic [7:0] status_register_value;

always @(posedge clk) begin
    if (reset) begin
        status_register_mask <= 8'd0;
        status_register_value <= 8'd0;
    end else if (next_byte_valid) begin
        unique case (command)
            CMD_SET_STATUS_REGISTER_MASK: status_register_mask <= next_byte;
            CMD_SET_STATUS_REGISTER_VALUE: status_register_value <= next_byte;
            default: /* nothing */;
        endcase
    end
end

logic [4:0] verify_delay;
logic [31:0] verify_timeout;

logic verify_pass;
always @(*) begin
    unique case (command)
        CMD_VERIFY_DATA: verify_pass = (cart_d_in == arg);
        CMD_VERIFY_STATUS_REGISTER: verify_pass = (cart_d_in & status_register_mask) == status_register_value;
        default: verify_pass = 1'b0;
    endcase
end

always @(posedge clk) begin
    if (state != S_EXEC_VERIFY) begin
        verify_delay <= 5'd24; // 400ns in 16.667ns ticks
        verify_timeout <= 32'd18_000; // 300usec in 16.667 ticks
        verify_state <= VS_PIN_RD_L;
    end else begin
        if (verify_timeout > 32'd0) begin
            verify_timeout <= verify_timeout - 32'd1;
        end
        unique case (verify_state)
            VS_PIN_RD_L: verify_state <= VS_DELAY;
            VS_DELAY: begin
                if (verify_delay > 5'd0) begin
                    verify_delay <= verify_delay - 5'd1;
                end else begin
                    verify_state <= VS_PIN_RD_H;
                end
            end
            VS_PIN_RD_H: begin
                if (verify_pass || (verify_timeout == 32'd0)) begin
                    verify_state <= VS_COMPLETE;
                end else begin
                    verify_delay <= 5'd24; // 400ns in 16.667ns ticks
                    verify_state <= VS_PIN_RD_L;
                end
            end
            default: ;
        endcase
    end
end

`define SET_PIN(TARGET, IDX) \
        if (arg[IDX + 4]) TARGET <= arg[IDX];
`define SET_TRISTATE_PIN(TARGET, IDX) \
        if (arg[IDX + 4]) begin \
            TARGET.oe <= 1'b1; \
            TARGET.value <= arg[IDX]; \
        end

always @(posedge clk) begin
    if (reset) begin
        cart_clk <= 1'b1;
        cart_wr <= 1'b1;
        cart_rd <= 1'b1;
        cart_cs <= 1'b1;
        cart_rst <= '{default: 0};
        cart_audio <= '{default: 0};

        cart_a <= 16'd0;
        cart_d_out <= 8'd0;
        cart_data_dir_e <= 1'b1; // read

        cart_a_oe <= 1'b1;
    end else begin
        unique case (command)
            CMD_SET_OUTPUT_ENABLE: begin
                if (arg[OE_AUDIO + 4]) begin
                    cart_audio.oe <= arg[OE_AUDIO];
                end
                if (arg[OE_DATA + 4]) begin
                    cart_data_dir_e <= ~arg[OE_DATA];
                end
                if (arg[OE_ADDRESS + 4]) begin
                    cart_a_oe <= arg[OE_ADDRESS];
                end
            end
            CMD_SET_PINS_A: begin
                `SET_PIN(cart_clk, SET_PINS_A_CLK);
                `SET_PIN(cart_wr, SET_PINS_A_WR);
                `SET_PIN(cart_rd, SET_PINS_A_RD)
                `SET_PIN(cart_cs, SET_PINS_A_CS)
            end
            CMD_SET_PINS_B: begin
                `SET_PIN(cart_a[15], SET_PINS_B_A15);
                `SET_TRISTATE_PIN(cart_rst, SET_PINS_B_RST);
                `SET_TRISTATE_PIN(cart_audio, SET_PINS_B_AUDIO);
            end
            CMD_SET_ADDRESS_MSB: begin
                cart_a[15:8] <= arg;
            end
            CMD_SET_ADDRESS_LSB: begin
                cart_a[7:0] <= arg;
            end
            CMD_SET_DATA: begin
                cart_d_out[7:0] <= arg;
            end
            default: ;
        endcase

        if (state == S_EXEC_VERIFY) begin
            unique case (verify_state)
                VS_PIN_RD_L: cart_rd <= 1'b0;
                VS_PIN_RD_H: cart_rd <= 1'b1;
                default: /* nothing */ ;
            endcase
        end
    end
end

always @(posedge clk) begin
    tx_valid <= 1'b0;
    tx_data <= 8'd0;

    if (!reset) begin
        unique case (command)
            CMD_PING: begin
                tx_valid <= 1'b1;
                tx_data <= ~arg;
            end
            CMD_GET_DATA: begin
                tx_valid <= 1'b1;
                tx_data <= cart_d_in;
            end
            CMD_VERIFY_DATA, CMD_VERIFY_STATUS_REGISTER: begin
                tx_valid <= (verify_state == VS_COMPLETE);
                tx_data <= verify_pass;
            end
            default: /* nop */ ;
        endcase
    end
end

endmodule