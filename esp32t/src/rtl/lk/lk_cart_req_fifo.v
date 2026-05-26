import lk_types::*;

module lk_cart_req_fifo_t(
    input wire clk,
    input wire reset,
    input wire enqueue,
    input wire dequeue,
    output reg empty,
    input cart_req_t in,
    output cart_req_t out
);

reg [2:0] read_idx;
reg [2:0] read_idx_next;
reg [2:0] write_idx;
reg [2:0] write_idx_next;

assign empty = (read_idx == write_idx);

always @(*) begin
    read_idx_next = read_idx;
    write_idx_next = write_idx;
    if (dequeue) read_idx_next = read_idx + 3'b1;
    if (enqueue) write_idx_next = write_idx + 3'b1;
end

always @(posedge clk) begin
    if (reset) begin
        read_idx <= 1'b0;
        write_idx <= 1'b0;
    end else begin
        read_idx <= read_idx_next;
        write_idx <= write_idx_next;
    end
end

cart_req_t reqs[0:7];

cart_req_t next_out;
always @(*) begin
    if (enqueue && empty) begin
        next_out = in;
    end else begin
        next_out = reqs[read_idx];
    end
end
always @(posedge clk) out <= next_out;
always @(posedge clk) if (enqueue) reqs[write_idx] <= in;

endmodule