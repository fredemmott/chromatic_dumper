module lk_serial_id_t(
    input clk,
    input enable,
    output reg complete,
    output reg tx_valid,
    output reg [7:0] tx_data
);

localparam ROM_LEN = 15;
localparam ROM_ADDR_WIDTH = $clog2(ROM_LEN);
reg [7:0] rom[0:ROM_LEN - 1];

initial begin
    rom[0] = "M"; // Microcode-based, not the LK protocol
    rom[1] = "i";
    rom[2] = "c";
    rom[3] = "r";
    rom[4] = "o";

    rom[5]  = "2"; // YYYY
    rom[6]  = "0";
    rom[7]  = "2";
    rom[8]  = "6";

    rom[9]  = "0"; // MM
    rom[10] = "6";

    rom[11] = "0"; // DD
    rom[12] = "1";

    rom[13] = "0"; // NN
    rom[14] = "1";
end

reg [ROM_ADDR_WIDTH-1:0] idx;

always @(posedge clk) begin
    tx_valid <= 1'b0;
    if (!enable) begin
        idx <= 0;
        complete <= 1'b0;
    end else if (idx < ROM_LEN) begin
        idx <= idx + 1;
        tx_valid <= 1'b1;
        tx_data <= rom[idx];
    end else begin
        complete <= 1'b1;
    end
end

endmodule