module FORWARD(
           input wire[31:0] r3_instr,
           input wire[31:0] r4_instr,
           input wire[31:0] r3_ans,
           input wire[31:0] r4_ans,
           input wire[31:0] A_in,
           input wire[31:0] B_in,
           input wire[31:0] r2_instr,

           output wire[31:0] A_out,
           output wire[31:0] B_out
       );

wire[4:0] A_addr;
wire[4:0] B_addr;
wire[4:0] r3_rd_addr;
wire[4:0] r4_rd_addr;

assign A_addr = r2_instr[19:15];
assign B_addr = r2_instr[24:20];

assign r3_rd_addr = (r3_instr[6:0] == 7'b1100011 || r3_instr[6:0] == 7'b0100011) ? 5'b00000 : r3_instr[11:7];
assign r4_rd_addr = (r4_instr[6:0] == 7'b1100011 || r4_instr[6:0] == 7'b0100011) ? 5'b00000 : r4_instr[11:7];

assign A_out = (A_addr == 5'b00000) ? A_in : (r3_rd_addr == A_addr) ? r3_ans : (r4_rd_addr == A_addr) ? r4_ans : A_in;
assign B_out = (B_addr == 5'b00000) ? B_in : (r3_rd_addr == B_addr) ? r3_ans : (r4_rd_addr == B_addr) ? r4_ans : B_in;

endmodule
