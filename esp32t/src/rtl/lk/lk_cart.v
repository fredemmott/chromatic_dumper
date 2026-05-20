import lk_types::*;

module lk_cart_t(
    input  wire  clk,
    input  bus_t in,
    output bus_t out
);

// Number of clock cycles CS/RD is asserted before latching data.
// At 60 MHz, 16 cycles ≈ 267 ns (GB min CS low = 200 ns).
localparam CART_RD_HOLD = 16;
// Write pulse width (WR low).  At 60 MHz, 10 cycles ≈ 167 ns.
localparam CART_WR_HOLD = 10;
// Address-to-CS setup cycles.
localparam CART_SETUP   = 4;

typedef enum logic [3:0] {
    C_IDLE,
    C_SETUP,  // address stable, dir set
    C_READ_PULSE_VAR,
    C_CSRD,   // CS/RD asserted
    C_WAIT,   // hold
    C_DONE,   // single-cycle done pulse
    C_WR_LOW, // write: WR low
    C_WR_HOLD,
    C_WR_HIGH // write: WR high + drive data
} state_t;
state_t state = C_IDLE;
reg [4:0] wait_cnt;

always @(posedge clk) begin
    out <= in;
    out.cart_complete <= 1'b0;

    if (in.reset) begin
        state    <= C_IDLE;
        out.cart <= cart_t'{
            clk: 1'b1,
            cs: 1'b1,
            rd: 1'b1,
            wr: 1'b1,
            rst: 1'b1,
            default: '0
        };
    end else begin
        let vars = in.vars;
        if (in.cart_request) state <= C_SETUP;

        // ─────────────────────────────────────────────────────────────────
        // Cart access state machine (runs every cycle, driven by pstate)
        // ─────────────────────────────────────────────────────────────────
        case (state)
        C_IDLE: ; // nothing

        C_SETUP: begin
            // Address and direction already set by caller two cycles ago.
            // Now assert CS with setup delay.
            wait_cnt <= CART_SETUP[4:0] - 5'd2;
            state <= C_CSRD;
        end

        C_CSRD: begin
            if (wait_cnt != 0) begin
                wait_cnt <= wait_cnt - 5'd1;
            end else begin
                if (in.cart.is_write) begin
                    out.cart.cs <= ~vars.dmg_write_cs_pulse;
                    wait_cnt <= CART_WR_HOLD[4:0] - 5'd1;
                    state    <= C_WR_LOW;
                end else begin
                    out.cart.cs <= ~vars.dmg_read_cs_pulse;
                    out.cart.rd <= 1'b0;
                    wait_cnt <= CART_RD_HOLD[4:0] - 5'd1;
                    state    <= C_WAIT;
                end
            end
        end

        C_WAIT: begin
            if (wait_cnt != 0) begin
                wait_cnt <= wait_cnt - 5'd1;
            end else begin
                out.cart.rd    <= 1'b1;
                out.cart.cs    <= 1'b1;
                state <= C_DONE;
            end
        end

        C_WR_LOW: begin
                // FIXME
            //case (cart_write_pulse_pins)
                //CART_WRITE_PULSE_PINS_DEFAULT: begin
                    out.cart.wr <= 1'b0;
                    out.cart.clk <= 1'b0;
                //end
                //CART_WRITE_PULSE_PINS_WR: cart_wr <= 1'b0;
                //CART_WRITE_PULSE_PINS_AUDIO: cart_audio <= 1'b0;
                //CART_WRITE_PULSE_PINS_NONE: begin
                //end
            //endcase

            if (wait_cnt != 0) begin
                wait_cnt <= wait_cnt - 5'd1;
            end else begin
                out.cart.clk <= 1'b1; // Raise clock WHILE WR is low
                wait_cnt <= CART_WR_HOLD[4:0] - 5'd1;
                state    <= C_WR_HOLD;
            end
        end

        C_WR_HOLD: begin
            // WR and CS are still low here
            if (wait_cnt != 0) begin
                wait_cnt <= wait_cnt - 5'd1;
            end else begin
                //FIXMEcase (cart_write_pulse_pins)
                //  CART_WRITE_PULSE_PINS_DEFAULT, CART_WRITE_PULSE_PINS_WR:
                  out.cart.wr <= 1'b1;
                //  CART_WRITE_PULSE_PINS_AUDIO: cart_audio <= 1'b1;
                //  CART_WRITE_PULSE_PINS_NONE: begin
                //  end
                //endcase
                out.cart.cs       <= 1'b1; // De-assert CS
                wait_cnt <= CART_WR_HOLD[4:0] - 5'd1;
                state    <= C_WR_HIGH;
            end
        end

        C_WR_HIGH: begin
            // Hold data/address stable for a moment after WR goes high
            if (wait_cnt != 0) begin
                wait_cnt <= wait_cnt - 5'd1;
            end else begin
                state      <= C_DONE;
            end
        end

        C_DONE: begin
            out.cart_complete <= 1'b1;
            state <= C_IDLE;
        end
        endcase // state
    end // ~reset
end // always

endmodule
