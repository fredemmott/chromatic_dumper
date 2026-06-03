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
    input  wire [7:0] cart_d_in,
    output reg  [7:0] cart_d_out,
    output reg        cart_audio
);

typedef enum {
    S_IDLE,
    S_SETUP,   // address stable, dir set
    S_CSRD,    // CS/RD asserted
    S_WAIT,    // hold for read
    S_WR_LOW,  // write: WR low
    S_WR_HIGH, // write: WR high + drive data
    S_SETUP_FOR_STATUS,
    S_WAIT_FOR_STATUS, // wait for the cartridge to indicate that a write was successful
    S_DONE     // single-cycle done pulse
} state_t;
state_t state;

reg current_req_is_write;
reg current_req_is_flash_write;
reg current_req_wait_for_status;
reg [7:0] current_req_data;
always @(posedge clk) begin
    req_started <= 1'b0;
    if (reset || (state == S_DONE)) begin
        cart_a <= 16'hFFFF;
        cart_d_out <= 8'hFF;
        cart_data_dir_e <= 1'b1; // read
        current_req_wait_for_status <= 1'b0;
    end else if ((state == S_IDLE) && req_valid) begin
        req_started <= 1'b1;

        cart_a <= req.address;
        cart_d_out <= req.data;
        cart_data_dir_e <= ~req.is_write;

        current_req_is_write <= req.is_write;
        current_req_is_flash_write <= (req.is_flash && req.is_write);
        current_req_wait_for_status <= req.wait_for_status;
        current_req_data <= req.data;
    end else if (state == S_SETUP_FOR_STATUS) begin
        cart_data_dir_e <= 1'b1; // is read
    end
end

reg req_pulse_audio;
reg req_pulse_wr;
reg req_pulse_rst;
always @(*) begin
    req_pulse_audio = 1'b0;
    req_pulse_wr = 1'b1;
    req_pulse_rst = 1'b0;

    if (current_req_is_flash_write) begin
        req_pulse_wr = 1'b0;
        unique case (var_flash_we_pin)
            FLASH_WE_PIN_AUDIO: req_pulse_audio = 1'b1;
            FLASH_WE_PIN_WR: req_pulse_wr = 1'b1;
            FLASH_WE_PIN_WR_AND_RESET: begin
                req_pulse_wr = 1'b1;
                req_pulse_rst = 1'b1;
            end
            default: ;
        endcase
    end
end

reg        cart_next_cs;
reg        cart_next_rd;
reg        cart_next_wr;
reg        cart_next_rst;
reg        cart_next_audio;
reg        cart_next_clk;
always @(*) begin
    // Pulsed
    cart_next_cs = 1'b1;
    cart_next_rd = 1'b1;
    cart_next_wr = 1'b1;
    cart_next_rst = 1'b1;
    cart_next_clk = 1'b1;
    cart_next_audio = hold_pin_audio;

    unique case (state)
        S_WAIT: begin
            cart_next_cs = ~var_dmg_read_cs_pulse;
            cart_next_rd = 1'b0;
        end
        S_WR_LOW: begin
            cart_next_clk = 1'b0;
            cart_next_wr = ~req_pulse_wr;
            cart_next_rst = ~req_pulse_rst;
            cart_next_cs = ~var_dmg_write_cs_pulse;
            cart_next_audio = hold_pin_audio ^ req_pulse_audio;
        end
        S_WAIT_FOR_STATUS: begin
            cart_next_cs = ~var_dmg_read_cs_pulse;
            cart_next_rd = 1'b0;
        end
        default: ;
    endcase
end

always @(posedge clk) begin
    if (reset) begin
        cart_cs <= 1'b1;
        cart_rd <= 1'b1;
        cart_wr <= 1'b1;
        cart_rst <= 1'b1;
        cart_clk <= 1'b1;
        cart_audio <= 1'b0;
    end else begin
        cart_cs <= cart_next_cs;
        cart_rd <= cart_next_rd;
        cart_wr <= cart_next_wr;
        cart_rst <= cart_next_rst;
        cart_clk <= cart_next_clk;
        cart_audio <= cart_next_audio;
    end
end

// Number of clock cycles to hold on a 59.605ns clock (hClk)
// Address-to-CS setup cycles.
// TODO current build is double timeouts
localparam CART_HOLD_CS_LOW = 2;
localparam CART_HOLD_RD_LOW = 4; // Aiming for 200ns
localparam CART_HOLD_WR_LOW = 4; // Aiming for ~240ns
localparam CART_HOLD_WR_HIGH = 2; // Aiming for ~ 120ns

reg [2:0] wait_cnt;

state_t next_state;
always @(*) begin
    next_state = state;
    unique case(state)
        S_IDLE: if (req_valid) next_state = S_SETUP;
        S_SETUP: next_state = S_CSRD;
        S_CSRD: if (wait_cnt == 0) next_state = (current_req_is_write ? S_WR_LOW : S_WAIT);
        S_WAIT: if (wait_cnt == 0) next_state = S_DONE;
        S_WR_LOW: if (wait_cnt == 0) next_state = S_WR_HIGH;
        S_WR_HIGH: begin
            if (wait_cnt == 0) begin
                if (current_req_wait_for_status) begin
                    next_state = S_SETUP_FOR_STATUS;
                end else begin
                    next_state = S_DONE;
                end
            end
        end
        S_SETUP_FOR_STATUS: if (wait_cnt == 0) next_state = S_WAIT_FOR_STATUS;
        S_WAIT_FOR_STATUS: begin
            if (wait_cnt == 0) begin
                if (current_req_data[7] == cart_d_in[7]) next_state = S_DONE;
                else next_state = S_SETUP_FOR_STATUS;
            end
        end
        S_DONE: next_state = S_IDLE;
        default: ;
    endcase
end

wire is_transition = (state != next_state);
always @(posedge clk) begin
    if (reset) begin
        wait_cnt <= 3'd0;
    end else if (wait_cnt > 3'd0) begin
        wait_cnt <= wait_cnt - 3'd1;
    end else if (is_transition) begin
        unique case (next_state)
            // - 1 as it's going to take us a cycle to get to the next state anyway
            S_CSRD:    wait_cnt <= CART_HOLD_CS_LOW - 1;
            S_WAIT:    wait_cnt <= CART_HOLD_RD_LOW - 1;
            S_WR_LOW:  wait_cnt <= CART_HOLD_WR_LOW - 1;
            S_WR_HIGH: wait_cnt <= CART_HOLD_WR_HIGH - 1;
            S_SETUP_FOR_STATUS: wait_cnt <= CART_HOLD_CS_LOW - 1;
            S_WAIT_FOR_STATUS: wait_cnt <= CART_HOLD_RD_LOW - 1;
            default:   wait_cnt <= 5'd0;
        endcase
    end
end

assign req_complete = (next_state == S_DONE);

always @(posedge clk) begin
    if (reset) begin
        state <= S_IDLE;
    end else begin
        state <= next_state;
    end
end

endmodule
