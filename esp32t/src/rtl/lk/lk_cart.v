import lk_types::*;

module lk_cart_t(
    input wire clk,
    input wire reset,
    input cart_req_t req,
    output reg req_complete,
    input cart_pins_t idle_pins,
    output cart_pins_t cart,
    input wire [1:0] var_flash_we_pin,
    input wire var_dmg_read_cs_pulse,
    input wire var_dmg_write_cs_pulse
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

cart_req_t current_req;
always @(posedge clk) begin
    unique case (state)
        S_IDLE: begin
            if (req.is_valid) current_req <= req;
            else current_req <= '{default: 0};
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

cart_pins_t next_cart;
always @(*) begin
    next_cart = idle_pins;
    if (current_req.is_valid) begin
        next_cart.address = current_req.address;
        next_cart.data = current_req.data;
        next_cart.data_dir_e = ~current_req.is_write;
        if (current_req.is_flash && current_req.is_write) begin
            next_cart.audio = (var_flash_we_pin == FLASH_WE_PIN_AUDIO);
            next_cart.wr = (var_flash_we_pin == FLASH_WE_PIN_WR) || (var_flash_we_pin == FLASH_WE_PIN_WR_AND_RESET);
            next_cart.rst = ~(var_flash_we_pin == FLASH_WE_PIN_WR_AND_RESET);
        end
    end

    unique case (state)
        S_WAIT: begin
            if (var_dmg_read_cs_pulse) next_cart.cs = ~idle_pins.cs;
            next_cart.rd = ~idle_pins.rd;
        end
        S_WR_LOW, S_WR_HOLD: begin
            next_cart.clk = ~idle_pins.clk;
            next_cart.wr = ~idle_pins.wr;
            if (var_dmg_write_cs_pulse) next_cart.cs = ~idle_pins.cs;
        end
        default: ;
    endcase
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
            S_IDLE: if (req.is_valid) next_state = S_SETUP;
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

always @(posedge clk) begin
    state <= next_state;
    cart <= next_cart;
    req_complete <= (state == S_DONE);

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
