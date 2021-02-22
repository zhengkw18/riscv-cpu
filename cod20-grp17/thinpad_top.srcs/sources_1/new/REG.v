module REG(
           //output
           output wire[31:0] A,					// A
           output wire[31:0] B,					// B
           //input
           input wire clk,				// 时钟
           input wire[31:0] r4_instr,
           input wire[31:0] r4_ans,
           input wire[31:0] r1_instr
       );

reg[31:0] registers[31:1];	// 寄存器组

wire[4:0] addr_a;
wire[4:0] addr_b;
assign addr_a = r1_instr[19:15];
assign addr_b = r1_instr[24:20];

wire we;
wire[4:0] waddr;
assign we = r4_instr[6:0] == 7'b1100011 || r4_instr[6:0] == 7'b0100011;
assign waddr = r4_instr[11:7];

always @(posedge clk) begin
    if(!we && waddr != 5'b00000) begin
        registers[waddr] <= r4_ans;
    end
end

assign A = (addr_a == 5'b00000) ? 32'h00000000 : (!we && waddr == addr_a) ? r4_ans : registers[addr_a];

assign B = (addr_b == 5'b00000) ? 32'h00000000 : (!we && waddr == addr_b) ? r4_ans : registers[addr_b];

endmodule
