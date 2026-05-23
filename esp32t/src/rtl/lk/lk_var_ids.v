package lk_var_ids;

typedef reg[9:0] var_id_t;

localparam var_id_t VAR_ID_INVALID = ~10'b0;

function var_id_t make_var16_id(
    input [7:0] size,
    input [7:0] key
);
    // Give all sizes are 1, 2, or 4, the lowest bit doesn't actually give us any information...
    // strip it off, then negate it so we can use null as a sentinel
    // size: 1 -> 001 -> 00 -> ~ 11
    // size: 2 -> 010 -> 01 -> ~ 10
    // size: 4 -> 100 -> 10 -> ~ 01
    begin
        make_var16_id = {~size[2:1], key[7:0]};
    end
endfunction

localparam VAR_ID_ADDRESS = make_var16_id(8'd4, 8'h00);
localparam VAR_ID_TRANSFER_SIZE = make_var16_id(8'd2, 8'h00);
localparam VAR_ID_STATUS_REGISTER = make_var16_id(8'd2, 8'h03);
localparam VAR_ID_CART_MODE = make_var16_id(8'd1, 8'h00);
localparam VAR_ID_DMG_ACCESS_MODE = make_var16_id(8'd1, 8'h01);
localparam VAR_ID_FLASH_WE_PIN = make_var16_id(8'd1, 8'h04);
localparam VAR_ID_DMG_READ_CS_PULSE = make_var16_id(8'd1, 8'h08);
localparam VAR_ID_DMG_WRITE_CS_PULSE = make_var16_id(8'd1, 8'h09);

endpackage