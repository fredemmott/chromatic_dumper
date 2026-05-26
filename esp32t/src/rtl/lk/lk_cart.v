import lk_types::*;

module lk_cart_t(
    input wire clk,
    input wire reset,

    input wire req_valid,
    input cart_req_t req,
    output reg req_started,
    output reg req_complete,

    input wire hold_pin_audio,

    input wire [1:0] var_flash_we_pin,
    input wire var_dmg_read_cs_pulse,
    input wire var_dmg_write_cs_pulse,

    output reg [15:0] cart_a,
    output reg        cart_clk,
    output reg        cart_cs,
    output reg        cart_rd,
    output reg        cart_wr,
    output reg        cart_rst,
    output reg        cart_data_dir_e,
    output reg [7:0]  cart_d_out,
    output reg        cart_audio
);

typedef enum {
    S_IDLE,
    S_SETUP,   // address stable, dir set
    S_CSRD,    // CS/RD asserted
    S_WAIT,    // hold for read
    S_WR_LOW,  // write: WR low
    S_WR_HOLD,
    S_WR_HIGH, // write: WR high + drive data
    S_DONE     // single-cycle done pulse
} state_t;
state_t state;

reg current_req_is_write;
reg current_req_is_flash_write;
always @(posedge clk) begin
    req_started <= 1'b0;
    if (reset || (state == S_DONE)) begin
        cart_a <= 16'hFFFF;
        cart_d_out <= 8'hFF;
        cart_data_dir_e <= 1'b1; // read
    end else if ((state == S_IDLE) && req_valid) begin
        req_started <= 1'b1;

        cart_a <= req.address;
        cart_d_out <= req.data;
        cart_data_dir_e <= ~req.is_write;

        current_req_is_write <= req.is_write;
        current_req_is_flash_write <= (req.is_flash && req.is_write);
    end
end

reg req_hold_audio;
reg req_hold_wr;
reg req_hold_rst;
always @(*) begin
    req_hold_audio = hold_pin_audio;
    req_hold_wr = 1'b1;
    req_hold_rst = 1'b1;

    if (current_req_is_flash_write) begin
        unique case (var_flash_we_pin)
            FLASH_WE_PIN_AUDIO: req_hold_audio = 1'b1;
            FLASH_WE_PIN_WR: req_hold_wr = 1'b0;
            FLASH_WE_PIN_WR_AND_RESET: begin
                req_hold_wr = 1'b0;
                req_hold_rst = 1'b0;
            end
            default: ;
        endcase
    end
end

always @(posedge clk) begin
    // Latched
    cart_audio <= req_hold_audio;
    cart_rst <= req_hold_rst;
end

reg        cart_next_clk;
reg        cart_next_cs;
reg        cart_next_rd;
reg        cart_next_wr;
always @(*) begin
    // Pulsed
    cart_next_clk = 1'b1;
    cart_next_cs = 1'b1;
    cart_next_rd = 1'b1;
    cart_next_wr = req_hold_wr;

    unique case (state)
        S_WAIT: begin
            if (var_dmg_read_cs_pulse) cart_next_cs = 1'b0;
            cart_next_rd = 1'b0;
        end
        S_WR_LOW, S_WR_HOLD: begin
            cart_next_clk = 1'b0;
            cart_next_wr = 1'b0;
            if (var_dmg_write_cs_pulse) cart_next_cs = 1'b0;
        end
        default: ;
    endcase
end

always @(posedge clk) begin
    if (reset) begin
        cart_clk <= 1'b1;
        cart_cs <= 1'b1;
        cart_rd <= 1'b1;
        cart_wr <= 1'b1;
    end else begin
        cart_clk <= cart_next_clk;
        cart_cs <= cart_next_cs;
        cart_rd <= cart_next_rd;
        cart_wr <= cart_next_wr;
    end
end

// Number of clock cycles CS/RD is asserted before latching data.
// At 60 MHz, 16 cycles ≈ 267 ns (GB min CS low = 200 ns).
localparam CART_RD_HOLD = 16;
// Write pulse width (WR low).  At 60 MHz, 10 cycles ≈ 167 ns.
localparam CART_WR_HOLD = 10;
// Address-to-CS setup cycles.
localparam CART_SETUP   = 4;

reg [4:0] wait_cnt;
reg [4:0] wait_cnt_next;

state_t next_state;

always @(*) begin
    wait_cnt_next = (wait_cnt > 5'd0) ? (wait_cnt - 5'd1) : 5'd0;
    unique case ({state, next_state})
        {S_SETUP,   S_CSRD}:    wait_cnt_next = CART_SETUP;
        {S_CSRD,    S_WAIT}:    wait_cnt_next = CART_RD_HOLD;
        {S_CSRD,    S_WR_LOW}:  wait_cnt_next = CART_WR_HOLD;
        {S_WR_LOW,  S_WR_HOLD}: wait_cnt_next = CART_WR_HOLD;
        {S_WR_HOLD, S_WR_HIGH}: wait_cnt_next = CART_WR_HOLD;
        default: ;
    endcase
end
always @(posedge clk) wait_cnt <= wait_cnt_next;


always @(*) begin
    next_state = state;
    unique case(state)
        S_IDLE: if (req_valid) next_state = S_SETUP;
        S_SETUP: next_state = S_CSRD;
        S_CSRD: if (wait_cnt == 0) next_state = (current_req_is_write ? S_WR_LOW : S_WAIT);
        S_WAIT: if (wait_cnt == 0) next_state = S_DONE;
        S_WR_LOW: if (wait_cnt == 0) next_state = S_WR_HOLD;
        S_WR_HOLD: if (wait_cnt == 0) next_state = S_WR_HIGH;
        S_WR_HIGH: if (wait_cnt == 0) next_state = S_DONE;
        S_DONE: next_state = S_IDLE;
        default: ;
    endcase
end

always @(posedge clk) req_complete <= (next_state == S_DONE);

always @(posedge clk) begin
    if (reset) begin
        state <= S_IDLE;
    end else begin
        state <= next_state;
    end
end

endmodule
