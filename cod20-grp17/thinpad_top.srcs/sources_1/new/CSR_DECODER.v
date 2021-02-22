module CSR_DECODER(
           //output
           output reg[2:0] op_code,
           //input
           input wire[31:0] ins
       );

//opcode
localparam
    I_4 = 7'b1110011;           // csrrc, csrrs, csrrw, ebreak, ecall, mret
//func3
localparam
    CSRRC_3 = 3'b011,
    CSRRS_3 = 3'b010,
    CSRRW_3 = 3'b001,
    ECALL_EBREAK_MRET_3 = 3'b000;

localparam
    CSRRW = 3'b000,
    CSRRS = 3'b001,
    CSRRC = 3'b010,
    ECALL = 3'b011,
    EBREAK = 3'b100,
    MRET = 3'b101,
    NOP = 3'b111;

localparam
    ECALL_12 = 12'b000000000000,
    EBREAK_12 = 12'b000000000001,
    MRET_12 = 12'b001100000010;

always @(*) begin
    case(ins[6:0])				// opcode
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
