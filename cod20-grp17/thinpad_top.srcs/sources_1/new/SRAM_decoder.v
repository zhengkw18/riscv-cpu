`default_nettype none

module SRAM_decoder(
           input wire clk,           //时钟输入

           input wire clock_btn,         //BTN5手动时钟按钮�??????关，带消抖电路，按下时为1
           input wire reset_btn,         //BTN6手动复位按钮�??????关，带消抖电路，按下时为1

           input wire[31:0] r2_instruction,
           input wire[31:0] ALU_ans,
           input wire[31:0] ALU_B,
           input wire[31:0] r0_pc,

           input wire[31:0] SRAM_ans,
           input wire[31:0] satp,
           input wire status, // 0代表U 1代表M

           output wire write_base,
           output wire write_ext,
           output wire write_uart,
           output wire read_uart,
           output wire from_base,
           output wire from_ext,

           output wire[3:0] byte,
           output wire[19:0] SRAM_addr,
           output wire[31:0] SRAM_data,
           output wire mem_stall,
           output wire read_offset,

           output wire bram_we_n,
           output wire r3_ans_en
       );
reg[31:0] r2_instruction_reg;
reg[31:0] ALU_B_reg;
wire[6:0] opcode;
assign opcode = (state == check && ~wait_uart)? r2_instruction[6:0]: r2_instruction_reg[6:0];
localparam
    load_code = 7'b0000011,
    store_code = 7'b0100011;

wire[2:0] funct3;
assign funct3 = (state == check && ~wait_uart)? r2_instruction[14:12]: r2_instruction_reg[14:12];
localparam
    funct_LB = 3'b000,
    funct_LW = 3'b010,
    funct_SB = 3'b000,
    funct_SW = 3'b010;

reg[2:0] command;
localparam
    fetch = 0,
    lb = 1,
    lw = 2,
    sb = 3,
    sw = 4;

always@* begin
    if(done) begin
        command <= fetch;
    end
    else begin
        case(opcode)
            load_code: begin
                case(funct3)
                    funct_LB: command <= lb;
                    funct_LW: command <= lw;
                    default: command <= fetch;
                endcase
            end
            store_code: begin
                case(funct3)
                    funct_SB: command <= sb;
                    funct_SW: command <= sw;
                    default: command <= fetch;
                endcase
            end
            default: begin
                command <= fetch;
            end
        endcase
    end
end

reg done;
wire conflict_reg;
assign conflict_reg = (command != fetch);
reg mem_stall_reg;
assign mem_stall = mem_stall_reg;
//TODO

reg[9:0] VPN1_reg;
reg[31:0] PTE1_reg;
reg[9:0] VPN2_reg;
reg[31:0] PTE2_reg;
reg[2:0] state;
localparam
    check = 3'b000,
    wait_VPN1 = 3'b001,
    check_VPN1 = 3'b010,
    wait_VPN2 = 3'b011,
    check_VPN2 = 3'b100;

reg wait_uart;
reg[2:0] wait_uart_count;

always@(posedge clk or posedge reset_btn) begin
    if(reset_btn) begin
        done <= 1'b0;
        VPN1_reg <= 10'h3ff;
        VPN2_reg <= 10'h3ff;
        state <= check;
        wait_uart <= 1'b0;
        wait_uart_count <= 0;
    end
    else begin
        case(state)
            check: begin
                if(status == 1'b0) begin
                    if(addr_src[31:22] == VPN1_reg) begin
                        if(addr_src[21:12] == VPN2_reg) begin
                            state <= check;//直接计算
                            if(done) begin
                                done <= 1'b0;
                            end
                            else if(conflict_reg) begin
                                done <= 1'b1;
                            end
                        end
                        else begin
                            VPN2_reg <= addr_src[21:12];
                            state <= wait_VPN2;//已匹配VPN1
                            addr_src_reg <= addr_src;
                            r2_instruction_reg <= r2_instruction;
                            ALU_B_reg <= ALU_B;
                        end
                    end
                    else begin
                        VPN1_reg <= addr_src[31:22];
                        VPN2_reg <= addr_src[21:12];
                        state <= wait_VPN1;
                        addr_src_reg <= addr_src;
                        r2_instruction_reg <= r2_instruction;
                        ALU_B_reg <= ALU_B;
                    end
                end
                else begin
                    state <= check;//直接计算
                    if(done) begin
                        done <= 1'b0;
                    end
                    else if(conflict_reg) begin
                        if(target == target_uart) begin
                            if(wait_uart) begin
                                if(wait_uart_count == 5) begin
                                    wait_uart <= 1'b0;
                                    done <= 1'b1;
                                    wait_uart_count <= 0;
                                end
                                else begin
                                    wait_uart_count <= wait_uart_count + 1;
                                end
                            end
                            else begin
                                addr_src_reg <= addr_src;
                                r2_instruction_reg <= r2_instruction;
                                ALU_B_reg <= ALU_B;
                                wait_uart <= 1'b1;
                            end
                        end 
                        else begin
                            done <= 1'b1;
                        end
                    end
                end
            end
            wait_VPN1: state <= check_VPN1;
            check_VPN1: begin
                // if((SRAM_ans & 32'he) == 32'h0) begin//非叶子节�??
                PTE1_reg <= SRAM_ans;
                state <= wait_VPN2;
                // end
                // else begin
                //     PTE2_reg <= SRAM_ans;
                //     state <= check;
                //     if(done) begin
                //         done <= 1'b0;
                //     end
                //     else if(conflict_reg) begin
                //         done <= 1'b1;
                //     end
                // end
            end
            wait_VPN2: state <= check_VPN2;
            check_VPN2: begin
                PTE2_reg <= SRAM_ans;
                state <= check;
                if(done) begin
                    done <= 1'b0;
                end
                else if(conflict_reg) begin
                    done <= 1'b1;
                end
            end
            default: state <= check;
        endcase
    end
end

wire[31:0] addr_src;
assign addr_src = (command == fetch)? r0_pc: ALU_ans;
reg[31:0] addr_src_reg;
reg[31:0] addr;
reg r3_ans_en_reg;
assign r3_ans_en = r3_ans_en_reg;
always@* begin//mem_stall and addr
    case(state)
        check: begin
            if(status == 1'b0) begin
                if(addr_src[31:22] == VPN1_reg) begin
                    if(addr_src[21:12] == VPN2_reg) begin//直接计算
                        mem_stall_reg <= (command != fetch);
                        r3_ans_en_reg <= 1'b1;
                        addr <= {PTE2_reg[29:10], addr_src[11:0]};
                    end
                    else begin//已匹配VPN1
                        mem_stall_reg <= 1'b1;
                        r3_ans_en_reg <= 1'b0;
                        addr <= {PTE1_reg[29:10], addr_src[21:12], 2'b00};
                    end
                end
                else begin
                    mem_stall_reg <= 1'b1;
                    r3_ans_en_reg <= 1'b0;
                    addr <= {satp[19:0], addr_src[31:22], 2'b00};
                end
            end
            else begin
                mem_stall_reg <= (command != fetch);
                r3_ans_en_reg <= 1'b1;
                if(wait_uart) begin
                    addr <= addr_src_reg;
                end
                else begin
                    addr <= addr_src;
                end
            end
        end
        wait_VPN1: begin
            mem_stall_reg <= 1'b1;
            r3_ans_en_reg <= 1'b0;
            addr <= addr_src_reg;
        end
        check_VPN1: begin
            // if((SRAM_ans & 32'he) == 32'h0) begin//非叶子节�??
            mem_stall_reg <= 1'b1;
            r3_ans_en_reg <= 1'b0;
            addr <= {SRAM_ans[29:10], addr_src_reg[21:12], 2'b00};
            // end
            // else begin
            //     mem_stall_reg <= (command != fetch);
            //     r3_ans_en_reg <= 1'b1;
            //     addr <= {SRAM_ans[29:10], addr_src[11:0]};
            // end
        end
        wait_VPN2: begin
            mem_stall_reg <= 1'b1;
            r3_ans_en_reg <= 1'b0;
            addr <= addr_src_reg;
        end
        check_VPN2: begin
            mem_stall_reg <= (command != fetch);
            r3_ans_en_reg <= 1'b1;
            addr <= {SRAM_ans[29:10], addr_src_reg[11:0]};
        end
        default: begin
            mem_stall_reg <= 1'b1;
            r3_ans_en_reg <= 1'b0;
            addr <= addr_src;
        end
    endcase
end
assign SRAM_addr = addr[21:2];

reg[3:0] byte_reg;
assign byte = r3_ans_en_reg? byte_reg: 4'b0000;
reg[31:0] SRAM_data_reg;
assign SRAM_data = SRAM_data_reg;
wire[31:0] ALU_B_wire;
assign ALU_B_wire = (state == check && ~wait_uart)? ALU_B: ALU_B_reg;
always@* begin
    case(command)
        sb: begin
            case(addr[1:0])
                2'b00: begin
                    byte_reg <= 4'b1110;
                    SRAM_data_reg <= {24'h000000, ALU_B_wire[7:0]};
                end
                2'b01: begin
                    byte_reg <= 4'b1101;
                    SRAM_data_reg <= {16'h0000, ALU_B_wire[7:0], 8'h00};
                end
                2'b10: begin
                    byte_reg <= 4'b1011;
                    SRAM_data_reg <= {8'h00, ALU_B_wire[7:0], 16'h0000};
                end
                2'b11: begin
                    byte_reg <= 4'b0111;
                    SRAM_data_reg <= {ALU_B_wire[7:0], 24'h000000};
                end
                default: begin
                    byte_reg <= 4'b1111;
                    SRAM_data_reg <= 32'h0000000000;
                end
            endcase
        end
        default: begin
            byte_reg <= 4'b0000;
            SRAM_data_reg <= ALU_B_wire;
        end
    endcase
end

reg write_sig;
always@* begin
    case(command)
        sb, sw: write_sig <= r3_ans_en_reg;
        default: write_sig <= 1'b0;
    endcase
end

reg[2:0] target;
localparam
    target_base = 3'b000,
    target_ext = 3'b001,
    target_uart = 3'b010,
    target_ureg = 3'b011,
    target_vga = 3'b100,
    target_nop = 3'b111;

always@* begin
    casez(addr)
        32'b1000000000??????????????????????: begin
            target <= target_base;
        end
        32'b1000000001??????????????????????: begin
            target <= target_ext;
        end
        32'b000100000000000000000000000000??: begin
            target <= target_uart;
        end
        32'b000100000000000000000000000001??: begin
            target <= target_ureg;
        end
        32'b0000000000000???????????????????:begin
            target <= target_vga;
        end
        default: begin
            target <= target_nop;
        end
    endcase
end

assign write_base = (target == target_base) && write_sig;
assign write_ext = (target == target_ext) && write_sig;
assign write_uart = (target == target_uart) && write_sig;
assign read_uart = (target == target_uart) && !write_sig;

assign from_base = ((target == target_base) || (target == target_uart)) && !write_sig;
assign from_ext = (target == target_ext) && !write_sig;

assign read_offset = (command == lb);

assign bram_we_n = ((target == target_vga) && write_sig);

endmodule
