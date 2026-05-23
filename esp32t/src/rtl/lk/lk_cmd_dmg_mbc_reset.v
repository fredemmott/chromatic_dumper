import lk_types::*;

module lk_cmd_dmg_mbc_reset_t(
    input wire clk,
    input wire enable,
    output reg complete,
    output cart_req_t cart_req,
    input reg cart_complete
);
    // Thanks to the gbdev.io pandocs: https://gbdev.io/pandocs/MBCs.html
    typedef enum {
        // 0x0000 <= 0x00
        S_RAM_DISABLE,
        // 0x2000 <= 0x01
        S_ROM_BANK_SEL_LOW,
        // 0x3000 <= 0x00
        // MBC1: same as BANK_SEL_LOW, but MBC1 treats 0x00 selection as 0x01
        // MBC3: sets all 7 bits, but also treats 0x00 selection as 0x01
        // MBC5: set high bits of bank
        S_ROM_BANK_SEL_HIGH,
        // 0x4000 <= 0
        // MBC1:
        // - usually RAM bank select
        // - also ROM bank number for some MBC multi-cart
        // MBC3: as for MBC1
        S_RAM_BANK_SEL,
        // 0x6000 <= 0
        // MBC1: 0 is 'simple' bank 0 ROM+SRAM (default), 1 is 'advanced' (0x4000 register is live)
        S_BANK_MODE_SEL,
        S_CART_WAIT,
        S_COMPLETE
    } state_t;
    localparam state_t S_INIT = S_RAM_DISABLE;
    state_t state;

    function cart_req_t make_req(input [15:0] address, input [7:0] data);
        return '{
            valid: 1'b1,
            is_write: 1'b1,
            address: address,
            data: data
        };
    endfunction

    always @(*) begin
        cart_req = '{default: 0};
        unique case (state)
            S_RAM_DISABLE: cart_req = make_req(16'h0000, 8'd0);
            S_ROM_BANK_SEL_LOW: cart_req = make_req(16'h2000, 8'd1);
            S_ROM_BANK_SEL_HIGH: cart_req = make_req(16'h3000, 8'd0);
            S_RAM_BANK_SEL: cart_req = make_req(16'h4000, 8'd0);
            S_BANK_MODE_SEL: cart_req = make_req(16'h6000, 8'd0);
            default: ;
        endcase
    end

    state_t state_next;
    // Used for S_CART_WAIT
    state_t state_on_cart_complete = S_COMPLETE;
    always @(*) begin
        complete = 1'b0;
        state_next = state;
        state_on_cart_complete = S_COMPLETE;
        if (!enable) begin
            state_next = S_INIT;
        end else begin
            unique case(state)
                S_RAM_DISABLE: begin
                    state_next = S_CART_WAIT;
                    state_on_cart_complete = S_ROM_BANK_SEL_LOW;
                end
                S_ROM_BANK_SEL_LOW: begin
                    state_next = S_CART_WAIT;
                    state_on_cart_complete = S_ROM_BANK_SEL_HIGH;
                end
                S_ROM_BANK_SEL_HIGH: begin
                    state_next = S_CART_WAIT;
                    state_on_cart_complete = S_RAM_BANK_SEL;
                end
                S_RAM_BANK_SEL: begin
                    state_next = S_CART_WAIT;
                    state_on_cart_complete = S_BANK_MODE_SEL;
                end
                S_BANK_MODE_SEL: begin
                    state_next = S_CART_WAIT;
                    state_on_cart_complete = S_COMPLETE;
                end
                S_CART_WAIT: begin
                    if (cart_complete) state_next = state_on_cart_complete;
                end
                S_COMPLETE: complete = 1;
                default: ;
            endcase
        end
    end
    always @(posedge clk) state <= state_next;
endmodule