import lk_types::*;
import lk_var_ids::*;

module lk_cmd_set_variable_t(
    input  wire       clk,
    input  wire       enable,
    output reg        complete,
    input  wire       rx_valid,
    input  wire [7:0] rx_data,
    input  vars_t     vars_in,
    output vars_t     vars_out
);

reg rx_valid_r;
reg [7:0] rx_data_r;
always @(posedge clk) begin
    rx_valid_r <= rx_valid;
    rx_data_r <= rx_data;
end

reg [3:0] idx;
reg [7:0] var_size;
reg [7:0] var_key;
reg [15:0] var_value;

var_id_t var_id;
assign var_id = make_var16_id(var_size, var_key);

vars_t vars_out_next;
always @(*) begin
    vars_out_next = vars_in;
    begin
        unique case (var_id)
            VAR_ID_ADDRESS: vars_out_next.address = var_value;
            VAR_ID_TRANSFER_SIZE: vars_out_next.transfer_size = var_value;
            VAR_ID_STATUS_REGISTER: vars_out_next.status_register = var_value[7:0];
            VAR_ID_CART_MODE: vars_out_next.cart_mode = var_value[7:0];
            VAR_ID_DMG_ACCESS_MODE: vars_out_next.dmg_access_mode = var_value[7:0];
            VAR_ID_FLASH_WE_PIN: vars_out_next.flash_we_pin = var_value[1:0];
            VAR_ID_DMG_READ_CS_PULSE: vars_out_next.dmg_read_cs_pulse = var_value[0];
            VAR_ID_DMG_WRITE_CS_PULSE: vars_out_next.dmg_write_cs_pulse = var_value[0];
            default: ;
        endcase
    end
end
always @(posedge clk) vars_out <= vars_out_next;

always @(posedge clk) begin
    if (!enable) begin
        idx <= 0;
    end else if (rx_valid_r) begin
        // byte [0]      size
        //      [1..4]   key (first 3 bytes unused)
        //      [5..8]   value (first 2 bytes unused)
        idx <= idx + 1'b1;
        unique case (idx)
            0: var_size <= rx_data_r;
            4: var_key <= rx_data_r;
            7: var_value[15:8] <= rx_data_r;
            8: var_value[7:0] <= rx_data_r;
            default: ;
        endcase
    end
end
always @(posedge clk) complete <= (idx >= 9);

endmodule