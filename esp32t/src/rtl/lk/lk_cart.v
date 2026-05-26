import lk_types::*;

module lk_cart_t(
    input wire clk,
    input wire reset,

    input wire req_valid,
    input cart_req_t req,
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
    S_SETUP,  // address stable, dir set
    S_CSRD,   // CS/RD asserted
    S_WAIT,   // hold
    S_DONE,   // single-cycle done pulse
    S_WR_LOW, // write: WR low
    S_WR_HOLD,
    S_WR_HIGH // write: WR high + drive data
} state_t;
state_t state;

reg current_req_valid;
cart_req_t current_req;
always @(posedge clk) begin
    unique case (state)
        S_IDLE: begin
            if (req_valid) begin
                current_req <= req;
                current_req_valid <= 1'b1;
            end else begin
                current_req_valid <= 1'b0;
                current_req <= '{default: 0};
            end
        end
        S_SETUP,
        S_CSRD,
        S_WAIT,
        S_WR_LOW,
        S_WR_HOLD,
        S_WR_HIGH,
        S_DONE: /* leave it alone; will clear when we go to idle next cycle */ ;
        default: current_req <= '{default: 0};
    endcase
end

reg [15:0] cart_next_a;
reg        cart_next_clk;
reg        cart_next_cs;
reg        cart_next_rd;
reg        cart_next_wr;
reg        cart_next_rst;
reg        cart_next_data_dir_e;
reg [7:0]  cart_next_d_out;
reg        cart_next_audio;
always @(*) begin
    cart_next_a = 16'hFFFF;
    cart_next_clk = 1'b1;
    cart_next_cs = 1'b1;
    cart_next_rd = 1'b1;
    cart_next_wr = 1'b1;
    cart_next_rst = 1'b1;
    cart_next_data_dir_e = 1'b1; // is read
    cart_next_d_out = 8'd0;
    cart_next_audio = hold_pin_audio;

    if (current_req_valid) begin
        cart_next_a = current_req.address;
        cart_next_d_out = current_req.data;
        cart_next_data_dir_e = ~current_req.is_write;
        if (current_req.is_flash && current_req.is_write) begin
            cart_next_audio = (var_flash_we_pin == FLASH_WE_PIN_AUDIO);
            cart_next_wr = (var_flash_we_pin == FLASH_WE_PIN_WR) || (var_flash_we_pin == FLASH_WE_PIN_WR_AND_RESET);
            cart_next_rst = ~(var_flash_we_pin == FLASH_WE_PIN_WR_AND_RESET);
        end
    end

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
    cart_a <= cart_next_a;
    cart_clk <= cart_next_clk;
    cart_cs <= cart_next_cs;
    cart_rd <= cart_next_rd;
    cart_wr <= cart_next_wr;
    cart_rst <= cart_next_rst;
    cart_data_dir_e <= cart_next_data_dir_e;
    cart_d_out <= cart_next_d_out;
    cart_audio <= cart_next_audio;
end

// Number of clock cycles CS/RD is asserted before latching data.
// At 60 MHz, 16 cycles ≈ 267 ns (GB min CS low = 200 ns).
localparam CART_RD_HOLD = 16;
// Write pulse width (WR low).  At 60 MHz, 10 cycles ≈ 167 ns.
localparam CART_WR_HOLD = 10;
// Address-to-CS setup cycles.
localparam CART_SETUP   = 4;
reg [4:0] wait_cnt;

state_t next_state;
always @(*) begin
    next_state = state;
    if (reset) begin
        next_state = S_IDLE;
    end else begin
        unique case(state)
            S_IDLE: if (current_req_valid) next_state = S_SETUP;
            S_SETUP: next_state = S_CSRD;
            S_CSRD: if (wait_cnt == 0) next_state = (current_req.is_write ? S_WR_LOW : S_WAIT);
            S_WAIT: if (wait_cnt == 0) next_state = S_DONE;
            S_WR_LOW: if (wait_cnt == 0) next_state = S_WR_HOLD;
            S_WR_HOLD: if (wait_cnt == 0) next_state = S_WR_HIGH;
            S_WR_HIGH: if (wait_cnt == 0) next_state = S_DONE;
            S_DONE: next_state = S_IDLE;
            default: ;
        endcase
    end
end
always @(posedge clk) req_complete <= (next_state == S_DONE);

always @(posedge clk) begin
    state <= next_state;

    if (!reset) begin
        unique case (state)
            S_IDLE: ;
            S_SETUP: begin
                // Address and direction already set by caller one cycles ago.
                // Now assert CS with setup delay.
                wait_cnt <= CART_SETUP - 5'd1;
            end

            S_CSRD: begin
                if (wait_cnt != 0) begin
                    wait_cnt <= wait_cnt - 5'd1;
                end else begin
                    if (current_req.is_write) begin
                        wait_cnt <= CART_WR_HOLD - 5'd1;
                    end else begin
                        wait_cnt <= CART_RD_HOLD - 5'd1;
                    end
                end
            end

            S_WAIT: wait_cnt <= wait_cnt - 5'd1;

            S_WR_LOW: begin
                if (wait_cnt != 0) wait_cnt <= wait_cnt - 5'd1;
                else wait_cnt <= CART_WR_HOLD - 5'd1;
            end

            S_WR_HOLD: begin
                // WR and CS are still low here
                if (wait_cnt != 0) begin
                    wait_cnt <= wait_cnt - 5'd1;
                end else begin
                    wait_cnt <= CART_WR_HOLD - 5'd1;
                end
            end

            S_WR_HIGH: begin
                // Hold data/address stable for a moment after WR goes high
                if (wait_cnt != 0) begin
                    wait_cnt <= wait_cnt - 5'd1;
                end
            end

            S_DONE: ;
            default: ;
        endcase // state
    end // ~reset
end // always

endmodule
