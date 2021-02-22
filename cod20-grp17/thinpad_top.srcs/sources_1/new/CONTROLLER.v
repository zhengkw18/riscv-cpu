module CONTROLLER(
           input wire clk,				// æ—¶é�?��?
           input wire reset_btn,
           input wire[31:0] current_pc,
           input wire[31:0] r1_pc,
           input wire[31:0] r2_pc,
           input wire r2_jmp,
           input wire[31:0] pc_plus_imm,
           input wire ALU_jmp,
           input wire stall,

           output wire is_nop,             //1则炸掉前两级流水�?
           output wire[31:0] next_pc,
           output wire jmp
       );

reg[31:0] pcs[7:0];
reg[31:0] next_pcs[7:0];
reg[7:0] mask;
wire[7:0] selector;
assign selector[0] = (r2_pc == pcs[0]) ? 1'b1 : 1'b0;
assign selector[1] = (r2_pc == pcs[1]) ? 1'b1 : 1'b0;
assign selector[2] = (r2_pc == pcs[2]) ? 1'b1 : 1'b0;
assign selector[3] = (r2_pc == pcs[3]) ? 1'b1 : 1'b0;
assign selector[4] = (r2_pc == pcs[4]) ? 1'b1 : 1'b0;
assign selector[5] = (r2_pc == pcs[5]) ? 1'b1 : 1'b0;
assign selector[6] = (r2_pc == pcs[6]) ? 1'b1 : 1'b0;
assign selector[7] = (r2_pc == pcs[7]) ? 1'b1 : 1'b0;

always @(*) begin
    case(selector)
        8'b00000001: begin
            mask <= 8'b00000000;
        end
        8'b00000010: begin
            mask <= 8'b00000001;
        end
        8'b00000100: begin
            mask <= 8'b00000011;
        end
        8'b00001000: begin
            mask <= 8'b00000111;
        end
        8'b00010000: begin
            mask <= 8'b00001111;
        end
        8'b00100000: begin
            mask <= 8'b00011111;
        end
        8'b01000000: begin
            mask <= 8'b00111111;
        end
        8'b10000000: begin
            mask <= 8'b01111111;
        end
        default: begin
            mask <= 8'b11111111;
        end
    endcase
end
// wire[7:0] mask;
// assign mask = (r2_pc == pcs[0]) ? 8'b00000000:
//        (r2_pc == pcs[1]) ? 8'b00000001:
//        (r2_pc == pcs[2]) ? 8'b00000011:
//        (r2_pc == pcs[3]) ? 8'b00000111:
//        (r2_pc == pcs[4]) ? 8'b00001111:
//        (r2_pc == pcs[5]) ? 8'b00011111:
//        (r2_pc == pcs[6]) ? 8'b00111111:
//        (r2_pc == pcs[7]) ? 8'b01111111:
//        8'b11111111;



reg[31:0] next_pc_reg;
reg jmp_reg;

localparam invalid = 32'hffffffff;

integer i1, i2;

wire err;
assign err = (ALU_jmp && !r2_jmp) || (!ALU_jmp && r2_jmp) || (ALU_jmp && r2_jmp && r1_pc != pc_plus_imm);
assign is_nop = err;

assign next_pc = err ? (ALU_jmp ? pc_plus_imm : r2_pc + 4) : next_pc_reg;
assign jmp = jmp_reg;

always @(posedge clk or posedge reset_btn) begin
    if(reset_btn) begin
        for(i1 = 0; i1 < 8; i1 = i1 + 1) begin
            pcs[i1] <= invalid;
        end
    end
    else if(!stall) begin
        if(r2_jmp) begin
            if(ALU_jmp) begin //??????????
                for(i1 = 0; i1 < 7; i1 = i1 + 1) begin
                    if(mask[i1]) begin
                        pcs[i1 + 1] <= pcs[i1];
                        next_pcs[i1 + 1] <= next_pcs[i1];
                    end
                end
                pcs[0] <= r2_pc;
                next_pcs[0] <= pc_plus_imm;
            end
            else begin //??
                for(i1 = 0; i1 < 7; i1 = i1 + 1) begin
                    if(!mask[i1]) begin
                        pcs[i1] <= pcs[i1 + 1];
                        next_pcs[i1] <= next_pcs[i1 + 1];
                    end
                end
                pcs[7] <= invalid;
            end
        end
        else if(!r2_jmp && ALU_jmp) begin //????????
            for(i1 = 0; i1 < 7; i1 = i1 + 1) begin
                pcs[i1 + 1] <= pcs[i1];
                next_pcs[i1 + 1] <= next_pcs[i1];
            end
            pcs[0] <= r2_pc;
            next_pcs[0] <= pc_plus_imm;
        end
    end
end

// always @(*) begin
//     next_pc_reg = current_pc + 4;
//     jmp_reg = 1'b0;
//     for(i2 = 7; i2 >= 0; i2 = i2 - 1) begin
//         next_pc_reg = (current_pc == pcs[i2]) ? next_pcs[i2] : next_pc_reg;
//         jmp_reg = (current_pc == pcs[i2]) ? 1'b1 : jmp_reg;
//     end
// end

wire[7:0] current_selector;
assign current_selector[0] = (current_pc == pcs[0]) ? 1'b1 : 1'b0;
assign current_selector[1] = (current_pc == pcs[1]) ? 1'b1 : 1'b0;
assign current_selector[2] = (current_pc == pcs[2]) ? 1'b1 : 1'b0;
assign current_selector[3] = (current_pc == pcs[3]) ? 1'b1 : 1'b0;
assign current_selector[4] = (current_pc == pcs[4]) ? 1'b1 : 1'b0;
assign current_selector[5] = (current_pc == pcs[5]) ? 1'b1 : 1'b0;
assign current_selector[6] = (current_pc == pcs[6]) ? 1'b1 : 1'b0;
assign current_selector[7] = (current_pc == pcs[7]) ? 1'b1 : 1'b0;

always @(*) begin
    case(current_selector)
        8'b00000001: begin
            next_pc_reg <= next_pcs[0];
            jmp_reg <= 1'b1;
        end
        8'b00000010: begin
            next_pc_reg <= next_pcs[1];
            jmp_reg <= 1'b1;
        end
        8'b00000100: begin
            next_pc_reg <= next_pcs[2];
            jmp_reg <= 1'b1;
        end
        8'b00001000: begin
            next_pc_reg <= next_pcs[3];
            jmp_reg <= 1'b1;
        end
        8'b00010000: begin
            next_pc_reg <= next_pcs[4];
            jmp_reg <= 1'b1;
        end
        8'b00100000: begin
            next_pc_reg <= next_pcs[5];
            jmp_reg <= 1'b1;
        end
        8'b01000000: begin
            next_pc_reg <= next_pcs[6];
            jmp_reg <= 1'b1;
        end
        8'b10000000: begin
            next_pc_reg <= next_pcs[7];
            jmp_reg <= 1'b1;
        end
        default: begin
            next_pc_reg <= current_pc + 4;
            jmp_reg <= 1'b0;
        end
    endcase
end
endmodule
