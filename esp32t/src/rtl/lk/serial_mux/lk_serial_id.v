module lk_serial_id_t(
    input clk,
    input enable,
    output reg complete,
    output reg tx_valid,
    output reg [7:0] tx_data
);

localparam ROM_LEN = 12;
localparam ROM_ADDR_WIDTH = $clog2(ROM_LEN);
reg [7:0] rom[0:ROM_LEN - 1];

initial begin
    rom[0] = "M"; // Microcode-based, not the LK protocol
    rom[1] = "i";
    rom[2] = "c";
    rom[3] = "r";
    rom[4] = "o";

    // Our version timestamp - BCD YYYY-MM-DD
    rom[5] = 8'h20; // YYYY
    rom[6] = 8'h26;
    rom[7] = 8'h06; // MM
    rom[8] = 8'h03; // DD

    // If we do multiple builds on the same day...
    rom[9] = 8'd01; // Revision
    //         ^ NOT BCD!

    // Upstream (ModRetro) version number
    rom[10] = 8'd18;
    //          ^ NOT BCD!
    rom[11] = 8'd08;
    //          ^ NOT BCD!
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