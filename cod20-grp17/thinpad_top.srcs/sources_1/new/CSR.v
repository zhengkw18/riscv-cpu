module CSR(
           //output
           output wire[31:0] csr_ans,
           output wire[31:0] csr_satp,
           output wire csr_status,
           //input
           input wire clk,				// ??
           input wire reset_btn,
           input wire[31:0] r1_instr,
           input wire[31:0] r2_instr,
           input wire[31:0] r2_csr_ans,
           input wire[31:0] A,
           input wire[31:0] pc,
           input wire stall
       );

reg[31:0] mtvec;
reg[31:0] mscratch;
reg[31:0] mepc;
reg[31:0] mcause;
reg[31:0] mstatus;
reg[31:0] satp;
assign csr_satp = (csr_to_write == code_satp) ? write_data : satp;

reg status; // 0??Uù?? 1??Mù??
assign csr_status = (r2_opcode == EBREAK || r2_opcode == ECALL)? 1'b1: (r2_opcode == MRET)? 0'b0: status;

wire[11:0] csr_to_write;

reg[31:0] write_data;

localparam
    code_mtvec = 12'h305,
    code_mscratch = 12'h340,
    code_mepc = 12'h341,
    code_mcause = 12'h342,
    code_mstatus = 12'h300,
    code_satp = 12'h180,
    code_mcause_mepc = 12'h641;

localparam
    MCAUSE_ECALL = 32'h00000008,
    MCAUSE_EBREAK = 32'h00000003;

localparam
    CSRRW = 3'b000,
    CSRRS = 3'b001,
    CSRRC = 3'b010,
    ECALL = 3'b011,
    EBREAK = 3'b100,
    MRET = 3'b101,
    NOP = 3'b111;

wire[2:0] r1_opcode;
wire[2:0] r2_opcode;

CSR_DECODER decoder1(
                .op_code(r1_opcode),
                .ins(r1_instr)
            );

CSR_DECODER decoder2(
                .op_code(r2_opcode),
                .ins(r2_instr)
            );

wire[11:0] r1_csr;

assign r1_csr = (r1_opcode == CSRRW || r1_opcode == CSRRS || r1_opcode == CSRRC) ? r1_instr[31:20]
       : (r1_opcode == ECALL || r1_opcode == EBREAK) ? code_mtvec
       : (r1_opcode == MRET) ? code_mepc
       : 12'b0;

assign csr_to_write = (r2_opcode == CSRRW || r2_opcode == CSRRS || r2_opcode == CSRRC) ? r2_instr[31:20]
       : (r2_opcode == ECALL || r2_opcode == EBREAK) ? code_mcause_mepc
       : 12'b0;

reg[31:0] r1_csr_data;
assign csr_ans = r1_csr_data;

always @(posedge clk or posedge reset_btn) begin
    if(reset_btn) begin
        status <= 1'b1;
    end
    else
        if(!stall) begin
            case (csr_to_write)
                code_mtvec:
                    mtvec <= write_data;
                code_mscratch:
                    mscratch <= write_data;
                code_mcause:
                    mcause <= write_data;
                code_mstatus:
                    mstatus <= write_data;
                code_satp:
                    satp <= write_data;
                code_mepc:
                    mepc <= write_data;
                code_mcause_mepc: begin
                    mcause <= write_data;
                    mepc <= pc;
                end
            endcase
            if(r2_opcode == EBREAK || r2_opcode == ECALL)begin
                status <= 1'b1;
            end
            else if (r2_opcode == MRET) begin
                status <= 1'b0;
            end
        end
end

always @(*) begin
    case (r1_csr)
        code_mtvec:
            r1_csr_data <= (csr_to_write == code_mtvec) ? write_data : mtvec;
        code_mscratch:
            r1_csr_data <= (csr_to_write == code_mscratch) ? write_data : mscratch;
        code_mepc:
            r1_csr_data <= (csr_to_write == code_mepc) ? write_data : (csr_to_write == code_mcause_mepc) ? pc : mepc;
        code_mcause:
            r1_csr_data <= (csr_to_write == code_mcause) ? write_data : mcause;
        code_mstatus:
            r1_csr_data <= (csr_to_write == code_mstatus || csr_to_write == code_mcause_mepc) ? write_data : mstatus;
        code_satp:
            r1_csr_data <= (csr_to_write == code_satp) ? write_data : satp;
        default:
            r1_csr_data <= 32'b0;
    endcase
end

always @(*) begin
    case (r2_opcode)
        CSRRW:
            write_data <= A;
        CSRRS:
            write_data <= r2_csr_ans | A;
        CSRRC:
            write_data <= r2_csr_ans & (~A);
        ECALL:
            write_data <= MCAUSE_ECALL;
        EBREAK:
            write_data <= MCAUSE_EBREAK;
        default:
            write_data <= 32'b0;
    endcase
end

endmodule
