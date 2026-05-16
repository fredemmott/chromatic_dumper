// Send an ack when complete
module lk_cmd_dmg_mbc_reset_t(
    input wire clk,
    input wire en,
    output wire complete,
    output reg [15:0] cart_a,
    output reg [7:0] cart_d_out,
    output reg cart_req,
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
    `define S_INIT S_RAM_DISABLE;
    state_t state = `S_INIT;
    // Used for S_CART_WAIT
    state_t state_on_cart_complete = S_COMPLETE;
    assign complete = (state == S_COMPLETE);

    initial begin
        cart_req = 0;
    end

    task start_write(
        reg [15:0] address,
        reg [7:0] data,
        state_t next
    );
        begin
            cart_a <= address;
            cart_d_out <= data;
            cart_req <= 1;
            state <= S_CART_WAIT;
            state_on_cart_complete <= next;
        end
    endtask

    always @(posedge clk) begin
        cart_req <= 1'b0;
        if (!en) begin
            cart_a <= 16'd0;
            cart_d_out <= 8'd0;
            state <= `S_INIT;
            state_on_cart_complete <= S_COMPLETE;
        end else begin
            case(state)
                S_RAM_DISABLE: start_write(16'h0000, 8'd0, S_ROM_BANK_SEL_LOW);
                S_ROM_BANK_SEL_LOW: start_write(16'h2000, 8'd1, S_ROM_BANK_SEL_HIGH);
                S_ROM_BANK_SEL_HIGH: start_write(16'h3000, 8'd0, S_RAM_BANK_SEL);
                S_RAM_BANK_SEL: start_write(16'h4000, 8'd0, S_BANK_MODE_SEL);
                S_BANK_MODE_SEL: start_write(16'h6000, 8'd0, S_COMPLETE);
                S_CART_WAIT: if (cart_complete) state <= state_on_cart_complete;
                S_COMPLETE: ;
                default: state <= S_COMPLETE;
            endcase
        end
    end
endmodule