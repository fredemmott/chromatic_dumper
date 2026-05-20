import lk_types::*;
import lk_var_ids::*;

module lk_cmd_set_variable_t(
    input  wire       clk,
    input  wire       reset,
    output reg        complete,
    input  wire       rx_valid,
    input  wire [7:0] rx_data,
    input  vars_t     vars_in,
    output vars_t     vars_out
);

reg [3:0] idx = 0;
reg [7:0] var_size;
var_id_t  var_id;
reg [15:0] var_value;

always @(posedge clk) begin
    if (reset) begin
        vars_out <= '{default: 0};
    end else begin
        vars_out <= vars_in;
        begin
            unique case (var_id)
                VAR_ID_ADDRESS: vars_out.address <= var_value;
                VAR_ID_TRANSFER_SIZE: vars_out.transfer_size <= var_value;
                VAR_ID_STATUS_REGISTER: vars_out.status_register <= var_value[7:0];
                VAR_ID_CART_MODE: vars_out.cart_mode <= var_value[7:0];
                VAR_ID_DMG_ACCESS_MODE: vars_out.dmg_access_mode <= var_value[7:0];
                VAR_ID_FLASH_WE_PIN: vars_out.flash_we_pin <= var_value[1:0];
                VAR_ID_DMG_READ_CS_PULSE: vars_out.dmg_read_cs_pulse <= var_value[0];
                VAR_ID_DMG_WRITE_CS_PULSE: vars_out.dmg_write_cs_pulse <= var_value[0];
                default: ;
            endcase
        end
    end
end

always @(posedge clk) begin
    if (reset) begin
        complete <= 1'b0;
        idx <= 0;
        var_id <= 0;
    end else if (rx_valid) begin
        // byte [0]      size
        //      [1..4]   key (first 3 bytes unused)
        //      [5..8]   value (first 2 bytes unused)
        idx <= idx + 1'b1;
        unique case (idx)
            0: var_size <= rx_data;
            4: var_id <= make_var16_id(var_size, rx_data);
            7: var_value[15:8] <= rx_data;
            8: begin
                var_value[7:0] <= rx_data;
                complete <= 1'b1;
            end
            default: ;
        endcase
    end
end

endmodule