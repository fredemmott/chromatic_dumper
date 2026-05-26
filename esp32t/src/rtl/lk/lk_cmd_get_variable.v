import lk_types::*;
import lk_var_ids::*;

module lk_cmd_get_variable_t(
    input  wire       clk,
    input  wire       enable,
    output reg        complete,
    input  wire       rx_valid,
    input  wire [7:0] rx_data,
    output reg        tx_valid,
    output reg  [7:0] tx_data,
    input  vars_t     vars
);

reg rx_valid_r;
reg [7:0] rx_data_r;
always @(posedge clk) begin
    rx_valid_r <= rx_valid;
    rx_data_r <= rx_data;
end

reg [7:0] var_size;
reg [7:0] var_key;
reg [15:0] var_value;

var_id_t var_id;
assign var_id = make_var16_id(var_size, var_key);

always @(*) begin
    var_value = 16'd0;
    begin
        unique case (var_id)
            VAR_ID_ADDRESS: var_value = vars.address;
            VAR_ID_TRANSFER_SIZE: var_value = vars.transfer_size;
            VAR_ID_STATUS_REGISTER: var_value = { 8'd0, vars.status_register };
            VAR_ID_CART_MODE: var_value = { 8'd0, vars.cart_mode };
            VAR_ID_DMG_ACCESS_MODE: var_value = { 8'd0, vars.dmg_access_mode };
            VAR_ID_FLASH_WE_PIN: var_value = { 15'd0, vars.flash_we_pin };
            VAR_ID_DMG_READ_CS_PULSE: var_value = { 15'd0, vars.dmg_read_cs_pulse };
            VAR_ID_DMG_WRITE_CS_PULSE: var_value = { 15'd0, vars.dmg_write_cs_pulse };
            default: ;
        endcase
    end
end

reg [2:0] rx_idx;
reg [2:0] tx_idx;

// byte [0]      size
//      [1..4]   key (first 3 bytes unused)
typedef enum {
    S_RX,
    S_TX,
    S_DONE
} state_t;
state_t state;

state_t next_state;
always @(*) begin
    next_state = state;
    if (!enable) begin
        next_state = S_RX;
    end else begin
        unique case (state)
            S_RX: if (rx_idx == 4) next_state = S_TX;
            S_TX: if (tx_idx == 3) next_state = S_DONE;
            S_DONE: next_state = S_RX;
            default: ;
        endcase
    end
end
always @(posedge clk) begin
    complete <= (next_state == S_DONE);
    state <= next_state;
end

always @(posedge clk) begin
    if (!enable) begin
        rx_idx <= 0;
    end else if ((state == S_RX) && rx_valid_r) begin
        rx_idx <= rx_idx + 1;
    end
end

always @(posedge clk) begin
    if (state == S_TX) begin
        tx_idx <= tx_idx + 1;
    end else begin
        tx_idx <= 0;
    end
end

always @(posedge clk) begin
    if ((state == S_RX) && rx_valid_r) begin
        unique case (rx_idx)
            0: var_size <= rx_data_r;
            4: var_key <= rx_data_r;
            default: ;
        endcase
    end
end

reg [7:0] tx_data_next;
always @(*) begin
    tx_data_next = 8'd0;
    unique case (tx_idx)
        2: tx_data_next = var_value[15:8];
        3: tx_data_next = var_value[7:0];
        default: ;
    endcase
end

always @(posedge clk) begin
    tx_valid <= (state == S_TX);
    tx_data <= tx_data_next;
end

endmodule