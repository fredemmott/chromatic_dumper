import lk_var_ids::*;
import lk_types::*;

module lk_var_storage_t(
    input wire clk,

    input wire clear_enable,
    output reg clear_complete,

    input wire set_enable,
    input var_id_t set_id,
    input wire [15:0] set_value,

    input wire get_enable,
    input var_id_t get_id,
    output reg [15:0] get_value,

    input wire advance_address_enable,

    output vars_t vars_o
);

// 7 bit IDs, still fits one slot
reg [15:0] storage [~7'd0:0];

always @(posedge clk) if (get_enable) get_value <= storage[get_id];

logic [15:0] next_address;
assign next_address = shadow.address + shadow.transfer_size;

var_id_t clear_idx;
always @(posedge clk) begin
    if (!clear_enable) begin
        clear_idx <= '0;
    end else if (clear_enable) begin
        clear_idx <= clear_idx + 1'd1;
    end
    clear_complete <= (clear_idx == ~var_id_t'(0));
end

vars_t shadow;
always @(posedge clk) begin
    if (advance_address_enable) begin
        storage[VAR_ID_ADDRESS] <= next_address;
        shadow.address <= next_address;
    end else if (set_enable) begin
        storage[set_id] <= set_value;
        unique case (set_id)
            VAR_ID_ADDRESS: shadow.address <= set_value;
            VAR_ID_TRANSFER_SIZE: shadow.transfer_size <= set_value;
            VAR_ID_FLASH_WE_PIN: shadow.flash_we_pin <= set_value[1:0];
            VAR_ID_DMG_READ_CS_PULSE: shadow.dmg_read_cs_pulse <= set_value[0];
            VAR_ID_DMG_WRITE_CS_PULSE: shadow.dmg_write_cs_pulse <= set_value[0];
            default: ;
        endcase
    end else if (clear_enable) begin
        storage[clear_idx] <= 16'd0;
        shadow <= '{default: 0};
    end
end
always @(posedge clk) vars_o <= shadow;

endmodule