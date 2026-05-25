import lk_types::*;

// TODO: mark valid
module lk_cmd_dmg_mbc_reset_t(
    input wire clk,
    input wire enable,
    output reg complete,
    output reg cart_req_valid,
    output cart_req_t cart_req,
    input reg cart_complete
);
    typedef struct {
        logic [15:0] address;
        logic [7:0] data;
    } command_t;
    /* This sequence should reset an MBC1, MBC3, or MBC5, using the same sequence of commands
     * for all of them; in some cases they have slightly different but useful behaviors, on others,
     * they're ignored
     *
     * Thanks to the gbdev.io pandocs: https://gbdev.io/pandocs/MBCs.html
     *
     * ROM_BANK_SEL_HIGH
     * -----------------
     *
     * MBC1: same as BANK_SEL_LOW, but MBC1 treats 0x00 selection as 0x01
     * MBC3: sets all 7 bits, but also treats 0x00 selection as 0x01
     * MBC5: set high bits of bank
     *
     * ... so, setting to 0 always works :)
     *
     * RAM_BANK_SEL
     * ------------
     *
     * MBC1:
     *  - usually RAM bank select
     *  - also ROM bank number for some MBC multi-cart
     * MBC3: RAM bank selectg
     *
     * BANK_MODE_SEL
     * -------------
     *
     * MBC1: 0 is 'simple' bank 0 ROM+SRAM (default), 1 is 'advanced' (0x4000 register is live)
     */
    localparam command_t C_RAM_DISABLE       = '{ 16'h0000, 8'h00 };
    localparam command_t C_ROM_BANK_SEL_LOW  = '{ 16'h2000, 8'h01 };
    localparam command_t C_ROM_BANK_SEL_HIGH = '{ 16'h3000, 8'h00 };
    localparam command_t C_RAM_BANK_SEL      = '{ 16'h4000, 8'h00 };
    localparam command_t C_BANK_MODE_SEL     = '{ 16'h6000, 8'h00 };
    localparam COMMAND_COUNT = 5;

    command_t command;
    always @(*) begin
        command = C_RAM_DISABLE;
        unique case (idx)
            0: command = C_RAM_DISABLE;
            1: command = C_ROM_BANK_SEL_LOW;
            2: command = C_ROM_BANK_SEL_HIGH;
            3: command = C_RAM_BANK_SEL;
            4: command = C_BANK_MODE_SEL;
            default: ;
        endcase
    end

    reg [2:0] req_waiting;
    always @(posedge clk) begin
        if (!enable) begin
            req_waiting <= COMMAND_COUNT;
        end else if (cart_complete) begin
            req_waiting <= req_waiting - 1'b1;
        end
    end

    typedef enum {
        S_INIT,
        S_REQ,
        S_WAIT,
        S_COMPLETE
    } state_t;
    state_t state;

    reg [2:0] idx;

    always @(posedge clk) begin
        if (!enable) begin
            idx <= 2'd0;
        end else begin
            if (cart_complete) idx <= idx + 1;
        end
    end

    always @(posedge clk) begin
        cart_req_valid <= 1'b0;
        cart_req <= '{default: 0};
        if (state == S_REQ) begin
            cart_req_valid <= 1'b1;
            cart_req <= '{
                is_write: 1'b1,
                is_flash: 1'b0,
                address: command.address,
                data: command.data
            };
        end
    end

    state_t next_state;
    always @(*) begin
        next_state = state;
        if (!enable) begin
            next_state = S_INIT;
        end else begin
            unique case (state)
                S_INIT: if (enable) next_state = S_REQ;
                S_REQ: if (idx == (COMMAND_COUNT - 1)) next_state = S_WAIT;
                S_WAIT: if (req_waiting == 0) next_state = S_COMPLETE;
                S_COMPLETE: ;
                default: ;
            endcase
        end
    end

    always @(posedge clk) begin
        state <= next_state;
        if (!enable) complete <= 1'b0;
        else complete <= (state == S_COMPLETE);
    end
endmodule