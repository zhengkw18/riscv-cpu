module IMM(
           input wire[31:0] instr,

           output wire[31:0] imm
       );

localparam
    type_I = 1,
    type_S = 2,
    type_B = 3,
    type_U = 4,
    type_J = 5;

wire[31:0] imms[5:0];
assign imms[0] = 32'h00000000;
assign imms[type_I] = {{20{instr[31]}},instr[31:20]};
assign imms[type_S] = {{20{instr[31]}},instr[31:25],instr[11:7]};
assign imms[type_B] = {{19{instr[31]}},instr[31],instr[7],instr[30:25],instr[11:8],1'b0};
assign imms[type_U] = {instr[31:12],12'h000};
assign imms[type_J] = {{11{instr[31]}},instr[31],instr[19:12],instr[20],instr[30:21],1'b0};

wire[2:0] type;
assign type = (instr[6:0] == 7'b1100111 || instr[6:0] == 7'b0000011 || instr[6:0] == 7'b0010011) ? type_I:
       (instr[6:0] == 7'b0100011) ? type_S:
       (instr[6:0] == 7'b1100011) ? type_B:
       (instr[6:0] == 7'b0010111 || instr[6:0] == 7'b0110111) ? type_U:
       (instr[6:0] == 7'b1101111) ? type_J : 0;
assign imm = imms[type];

endmodule
