module ALU(
           //output
           output wire[31:0] ans,		// 正经结果, pc+imm | ans
           output wire is_jumping,
           output wire is_bj,
           output wire[31:0] pc_imm,	// 传给上方controller, pc+imm
           //input
           input wire[31:0] instr,		// 指令
           input wire[31:0] pc,
           input wire[4:0] operator,
           input wire[31:0] dataA,
           input wire[31:0] dataB,
           input wire[31:0] imm,		// pc相关imm
           input wire[31:0] csr_ans
       );

localparam
    ADD = 5'b00000,
    AND = 5'b00001,
    OR = 5'b00010,
    XOR = 5'b00011,
    ADDI = 5'b00100,
    ANDI = 5'b00101,
    ORI = 5'b00110,
    SLLI = 5'b00111,
    SRLI = 5'b01000,
    AUIPC = 5'b01001,
    BEQ = 5'b01010,
    BNE = 5'b01011,
    JAL = 5'b01100,
    SB = 5'b01101,
    SW = 5'b01110,
    JALR = 5'b01111,
    LB = 5'b10000,
    LW = 5'b10001,
    LUI = 5'b10010,
    SBSET = 5'b10011,
    CLZ = 5'b10100,
    PCNT = 5'b10101,
    CSRRC = 5'b10110,
    CSRRS = 5'b10111,
    CSRRW = 5'b11000,
    ECALL = 5'b11001,
    EBREAK = 5'b11010,
    MRET = 5'b11011,
    NOP = 5'b11111;

reg is_j;
assign is_jumping = is_j;		// is_jumping
reg is_b_j;
assign is_bj = is_b_j;

assign pc_imm = (operator == ECALL || operator == EBREAK || operator == MRET) ? csr_ans : (operator == JALR) ? dataA + imm : pc + imm;	// pc_imm

always @(*) begin
    case(operator)
        BEQ:begin
            is_j <= (dataA == dataB);
            is_b_j <= 1'b1;
        end
        BNE:begin
            is_j <= !(dataA == dataB);
            is_b_j <= 1'b1;
        end
        JAL, JALR, ECALL, EBREAK, MRET:begin
            is_j <= 1'b1;
            is_b_j <= 1'b1;
        end
        default:begin
            is_j <= 1'b0;
            is_b_j <= 1'b0;
        end
    endcase
end								// compare

reg[31:0] answer;
reg[31:0] clz_counter, clz_a;	// clz?
reg[31:0] pcnt_counter;			// pcnt?

always @(*) begin
    case(operator)
        ADD:begin
            answer <= dataA + dataB;
        end
        ADDI:begin
            answer <= dataA + imm;
        end
        AND:begin
            answer <= dataA & dataB;
        end
        ANDI:begin
            answer <= dataA & imm;
        end
        OR:begin
            answer <= dataA | dataB;
        end
        ORI:begin
            answer <= dataA | imm;
        end
        SLLI:begin
            answer <= dataA << imm;
        end
        SRLI:begin
            answer <= dataA >> imm;
        end
        XOR:begin
            answer <= dataA ^ dataB;
        end
        SBSET:begin
            answer <= dataA | (1 << (dataB & 32'b11111));
        end
        CLZ:begin
            if(dataA == 0) begin
                answer <= 32;
            end
            else begin
                clz_counter <= 1;
                clz_a <= dataA;
                if((clz_a >> 16) == 0) begin
                    clz_counter = clz_counter + 16;
                    clz_a = clz_a << 16;
                end
                if((clz_a >> 24) == 0) begin
                    clz_counter = clz_counter + 8;
                    clz_a = clz_a << 8;
                end
                if((clz_a >> 28) == 0) begin
                    clz_counter = clz_counter + 4;
                    clz_a = clz_a << 4;
                end
                if((clz_a >> 30) == 0) begin
                    clz_counter = clz_counter + 2;
                    clz_a = clz_a << 2;
                end
                clz_counter = clz_counter - (clz_a >> 31);
                answer <= clz_counter;
            end
            // casez(dataA)
            //     32'b00000000000000000000000000000000:answer <= 32;
            //     32'b00000000000000000000000000000001:answer <= 31;
            //     32'b0000000000000000000000000000001?:answer <= 30;
            //     32'b000000000000000000000000000001??:answer <= 29;
            //     32'b00000000000000000000000000001???:answer <= 28;
            //     32'b0000000000000000000000000001????:answer <= 27;
            //     32'b000000000000000000000000001?????:answer <= 26;
            //     32'b00000000000000000000000001??????:answer <= 25;
            //     32'b0000000000000000000000001???????:answer <= 24;
            //     32'b000000000000000000000001????????:answer <= 23;
            //     32'b00000000000000000000001?????????:answer <= 22;
            //     32'b0000000000000000000001??????????:answer <= 21;
            //     32'b000000000000000000001???????????:answer <= 20;
            //     32'b00000000000000000001????????????:answer <= 19;
            //     32'b0000000000000000001?????????????:answer <= 18;
            //     32'b000000000000000001??????????????:answer <= 17;
            //     32'b00000000000000001???????????????:answer <= 16;
            //     32'b0000000000000001????????????????:answer <= 15;
            //     32'b000000000000001?????????????????:answer <= 14;
            //     32'b00000000000001??????????????????:answer <= 13;
            //     32'b0000000000001???????????????????:answer <= 12;
            //     32'b000000000001????????????????????:answer <= 11;
            //     32'b00000000001?????????????????????:answer <= 10;
            //     32'b0000000001??????????????????????:answer <= 9;
            //     32'b000000001???????????????????????:answer <= 8;
            //     32'b00000001????????????????????????:answer <= 7;
            //     32'b0000001?????????????????????????:answer <= 6;
            //     32'b000001??????????????????????????:answer <= 5;
            //     32'b00001???????????????????????????:answer <= 4;
            //     32'b0001????????????????????????????:answer <= 3;
            //     32'b001?????????????????????????????:answer <= 2;
            //     32'b01??????????????????????????????:answer <= 1;
            //     32'b1???????????????????????????????:answer <= 0;
            // endcase
        end								// 二分
        PCNT:begin
            pcnt_counter <= dataA;
            pcnt_counter = (pcnt_counter & 32'h55555555) + ((pcnt_counter >> 1) & 32'h55555555);
            pcnt_counter = (pcnt_counter & 32'h33333333) + ((pcnt_counter >> 2) & 32'h33333333);
            pcnt_counter = (pcnt_counter & 32'h0f0f0f0f) + ((pcnt_counter >> 4) & 32'h0f0f0f0f);
            pcnt_counter = (pcnt_counter & 32'h00ff00ff) + ((pcnt_counter >> 8) & 32'h00ff00ff);
            pcnt_counter = (pcnt_counter & 32'h0000ffff) + ((pcnt_counter >> 16) & 32'h0000ffff); // >>> ?
            answer <= pcnt_counter;
        end								// 分析
        SW, SB, LW, LB:begin
            answer <= dataA + imm;
        end
        LUI:begin
            answer <= imm;
        end
        AUIPC:begin
            answer <= pc + imm;			// AUIPC
        end
        JALR, JAL:begin
            answer <= pc + 4;
        end
        CSRRC, CSRRS, CSRRW:begin
            answer <= csr_ans;
        end
        default:begin
            answer <= 32'b0;
        end
    endcase
end

assign ans = answer;

endmodule
