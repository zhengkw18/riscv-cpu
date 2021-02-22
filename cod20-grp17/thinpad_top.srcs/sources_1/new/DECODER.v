module DECODER(
           //output
           output reg[4:0] op_code,
           //input
           input wire[31:0] ins
       );

//opcode
localparam
    R = 7'b0110011,				// add, and, or, xor, sbset
    I = 7'b0010011,				// addi, andi, ori, slli, srli, clz, pcnt
    U = 7'b0010111,				// auipc
    B = 7'b1100011,				// beq, bne
    J = 7'b1101111,				// jal
    S = 7'b0100011,				// sb, sw
    I_2 = 7'b1100111,			// jalr
    I_3 = 7'b0000011,			// lb, lw
    U_2 = 7'b0110111,			// lui
    I_4 = 7'b1110011;           // csrrc, csrrs, csrrw, ebreak, ecall, mret
//func3
localparam
    ADD_3 = 3'b000,
    AND_3 = 3'b111,
    OR_3 = 3'b110,
    XOR_3 = 3'b100,
    ADDI_3 = 3'b000,
    ANDI_3 = 3'b111,
    ORI_3 = 3'b110,
    SLLI_3 = 3'b001,
    SRLI_3 = 3'b101,
    BEQ_3 = 3'b000,
    BNE_3 = 3'b001,
    SB_3 = 3'b000,
    SW_3 = 3'b010,
    JALR_3 = 3'b000,
    LB_3 = 3'b000,
    LW_3 = 3'b010,
    SBSET_3 = 3'b001,
    CSRRC_3 = 3'b011,
    CSRRS_3 = 3'b010,
    CSRRW_3 = 3'b001,
    ECALL_EBREAK_MRET_3 = 3'b000;

localparam
    CLZ_5 = 5'b00000,
    PCNT_5 = 5'b00010;

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

localparam
    ECALL_12 = 12'b000000000000,
    EBREAK_12 = 12'b000000000001,
    MRET_12 = 12'b001100000010;

always @(*) begin
    case(ins[6:0])				// opcode
        R:begin
            case(ins[14:12])		// func3
                ADD_3:begin
                    op_code <= ADD;
                end
                AND_3:begin
                    op_code <= AND;
                end
                OR_3:begin
                    op_code <= OR;
                end
                XOR_3:begin
                    op_code <= XOR;
                end
                SBSET_3:begin		// sbset
                    op_code <= SBSET;
                end
                default:begin
                    op_code <= NOP;
                end
            endcase
        end
        I:begin
            case(ins[14:12])		// func3
                ADDI_3:begin
                    op_code <= ADDI;
                end
                ANDI_3:begin
                    op_code <= ANDI;
                end
                ORI_3:begin
                    op_code <= ORI;
                end
                SLLI_3:begin		// slli, clz, ctz
                    case(ins[24:20])
                        CLZ_5:begin
                            op_code <= CLZ;
                        end
                        PCNT_5:begin
                            op_code <= PCNT;
                        end
                        default:begin
                            op_code <= SLLI;
                        end
                    endcase
                end
                SRLI_3:begin
                    op_code <= SRLI;
                end
                default:begin
                    op_code <= NOP;
                end
            endcase
        end
        U:begin
            op_code <= AUIPC;
        end
        B:begin
            case(ins[14:12])		// func3
                BEQ_3:begin
                    op_code <= BEQ;
                end
                BNE_3:begin
                    op_code <= BNE;
                end
                default:begin
                    op_code <= NOP;
                end
            endcase
        end
        J:begin
            op_code <= JAL;
        end
        S:begin
            case(ins[14:12])		// func3
                SB_3:begin
                    op_code <= SB;
                end
                SW_3:begin
                    op_code <= SW;
                end
                default:begin
                    op_code <= NOP;
                end
            endcase
        end
        I_2:begin
            case(ins[14:12])		// func3
                JALR_3:begin
                    op_code <= JALR;
                end
                default:begin
                    op_code <= NOP;
                end
            endcase
        end
        I_3:begin
            case(ins[14:12])		// func3
                LB_3:begin
                    op_code <= LB;
                end
                LW_3:begin
                    op_code <= LW;
                end
                default:begin
                    op_code <= NOP;
                end
            endcase
        end
        U_2:begin
            op_code <= LUI;
        end
        I_4:begin
            case(ins[14:12])		// func3
                CSRRC_3:begin
                    op_code <= CSRRC;
                end
                CSRRS_3:begin
                    op_code <= CSRRS;
                end
                CSRRW_3:begin
                    op_code <= CSRRW;
                end
                ECALL_EBREAK_MRET_3:begin
                    case(ins[31:20])
                        ECALL_12:begin
                            op_code <= ECALL;
                        end
                        EBREAK_12:begin
                            op_code <= EBREAK;
                        end
                        MRET_12:begin
                            op_code <= MRET;
                        end
                        default:begin
                            op_code <= NOP;
                        end
                    endcase
                end
                default:begin
                    op_code <= NOP;
                end
            endcase
        end
        default:begin
            op_code <= NOP;
        end
    endcase
end

endmodule
