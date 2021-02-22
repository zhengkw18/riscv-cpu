`default_nettype none

module SRAM_old(
           input wire clk,           //时钟输入

           input wire clock_btn,         //BTN5手动时钟按钮�??关，带消抖电路，按下时为1
           input wire reset_btn,         //BTN6手动复位按钮�??关，带消抖电路，按下时为1

           input  wire[3:0]  touch_btn,  //BTN1~BTN4，按钮开关，按下时为1
           input  wire[31:0] dip_sw,     //32位拨码开关，拨到“ON”时�??1
        //    output wire[15:0] leds,       //16位LED，输出时1点亮
           output wire[7:0]  dpy0,       //数码管低位信号，包括小数点，输出1点亮
           output wire[7:0]  dpy1,       //数码管高位信号，包括小数点，输出1点亮

           //CPLD串口控制器信�??
           output wire uart_rdn,         //读串口信号，低有�??
           output wire uart_wrn,         //写串口信号，低有�??
           input wire uart_dataready,    //串口数据准备�??
           input wire uart_tbre,         //发�?�数据标�??
           input wire uart_tsre,         //数据发�?�完毕标�??

           //BaseRAM信号
           inout wire[31:0] base_ram_data,  //BaseRAM数据，低8位与CPLD串口控制器共�??
           output wire[19:0] base_ram_addr, //BaseRAM地址
           output wire[3:0] base_ram_be_n,  //BaseRAM字节使能，低有效。如果不使用字节使能，请保持�??0
           output wire base_ram_ce_n,       //BaseRAM片�?�，低有�??
           output wire base_ram_oe_n,       //BaseRAM读使能，低有�??
           output wire base_ram_we_n,       //BaseRAM写使能，低有�??

           //ExtRAM信号
           inout wire[31:0] ext_ram_data,  //ExtRAM数据
           output wire[19:0] ext_ram_addr, //ExtRAM地址
           output wire[3:0] ext_ram_be_n,  //ExtRAM字节使能，低有效。如果不使用字节使能，请保持�??0
           output wire ext_ram_ce_n,       //ExtRAM片�?�，低有�??
           output wire ext_ram_oe_n,       //ExtRAM读使能，低有�??
           output wire ext_ram_we_n,       //ExtRAM写使能，低有�??

           input wire[31:0] r3_instruction,
           input wire[31:0] r3_ans,
           input wire[31:0] r3_b,
           input wire[31:0] r3_pc,

           input wire[31:0] r0_pc,
           output wire[31:0] r0_instruction,

           output wire[31:0] r3_output_data,
           output wire mem_stall
       );

localparam NOP = 32'b00000000000000000000000000010011; // addi x0,x0,0

reg uart_state;
localparam
    wait_tbre = 0,
    wait_tsre = 1;

wire uart_can_write;
assign uart_can_write = (uart_state == wait_tsre) && uart_tsre;

reg write_uart;

always@(posedge clk or posedge reset_btn) begin
    if(reset_btn) begin
        uart_state <= wait_tsre;
    end
    else begin
        case(uart_state)
            wait_tbre: begin
                if(uart_tbre) begin
                    uart_state <= wait_tsre;
                end
            end
            wait_tsre: begin
                if(write_uart) begin
                    uart_state <= wait_tbre;
                end
            end
        endcase
    end
end

wire[6:0] opcode;
assign opcode = r3_instruction[6:0];
localparam
    load_code = 7'b0000011,
    store_code = 7'b0100011;

wire[2:0] funct3;
assign funct3 = r3_instruction[14:12];
localparam
    funct_LB = 3'b000,
    funct_LW = 3'b010,
    funct_SB = 3'b000,
    funct_SW = 3'b010;

wire[31:0] r3_data;
assign r3_data = r3_b;

reg[31:0] addr;

reg read_uart;
reg write_base;
reg write_ext;
assign base_ram_we_n = write_base? clk: 1'b1;
assign ext_ram_we_n = write_ext? clk: 1'b1;
assign uart_wrn = ~write_uart;
// assign uart_wrn = write_uart? clk: 1'b1;
assign base_ram_oe_n = write_base || use_uart;
assign ext_ram_oe_n = write_ext;
assign uart_rdn = ~read_uart;

wire use_uart;
assign use_uart = read_uart || write_uart;
assign base_ram_ce_n = use_uart;
assign ext_ram_ce_n = 1'b0;


reg[31:0] data_write;
assign base_ram_data = (write_base || write_uart)? data_write: 32'bz;
assign ext_ram_data = write_ext? data_write: 32'bz;

reg[3:0] byte_reg;
assign base_ram_be_n = byte_reg;
assign ext_ram_be_n = byte_reg;

reg stall_reg;
reg done;
assign mem_stall = stall_reg;

// assign leds = {command, 12'b10, stall_reg};

reg[31:0] r0_instruction_reg;
reg[31:0] r3_output_data_reg;

assign r0_instruction = r0_instruction_reg;
assign r3_output_data = r3_output_data_reg;

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
        addr <= r0_pc;
    end
    else begin
        case(opcode)
            load_code: begin
                case(funct3)
                    funct_LB: command <= lb;
                    funct_LW: command <= lw;
                    default: command <= fetch;
                endcase
                addr <= r3_ans;
            end
            store_code: begin
                case(funct3)
                    funct_SB: command <= sb;
                    funct_SW: command <= sw;
                    default: command <= fetch;
                endcase
                addr <= r3_ans;
            end
            default: begin
                command <= fetch;
                addr <= r0_pc;
            end
        endcase
    end
end

wire[31:0] uart_reg;
assign uart_reg = {16'h0000, 2'b00, uart_can_write, 4'b0000, uart_dataready, 8'h00};

reg[19:0] addr_reg;
assign base_ram_addr = addr_reg;
assign ext_ram_addr = addr_reg;

reg[31:0] tmp_read_reg;

always@* begin
    casez(addr)
        32'b1000000000??????????????????????: begin
            tmp_read_reg <= base_ram_data;
        end
        32'b1000000001??????????????????????: begin
            tmp_read_reg <= ext_ram_data;
        end
        32'b000100000000000000000000000000??: begin
            tmp_read_reg <= base_ram_data;
        end
        32'b000100000000000000000000000001??: begin
            tmp_read_reg <= uart_reg;
        end
        default: begin
            tmp_read_reg <= 32'b0;
        end
    endcase
end

always@* begin
    addr_reg <= addr[21:2];
    case(command)
        lb: begin
            stall_reg <= 1'b1;
            write_base <= 1'b0;
            write_ext <= 1'b0;
            write_uart <= 1'b0;
            byte_reg <= 4'b0000;
            r0_instruction_reg <= NOP;
            data_write <= 32'b0;

            casez(addr)
                32'b1000000000??????????????????????: begin
                    read_uart <= 1'b0;
                end
                32'b1000000001??????????????????????: begin
                    read_uart <= 1'b0;
                end
                32'b000100000000000000000000000000??: begin
                    read_uart <= 1'b1;
                end
                32'b000100000000000000000000000001??: begin
                    read_uart <= 1'b0;
                end
                default: begin
                    read_uart <= 1'b0;
                end
            endcase

            case(addr[1:0])
                2'b00: begin
                    if(tmp_read_reg[7]) begin
                        r3_output_data_reg <= {24'hffffff, tmp_read_reg[7:0]};
                    end
                    else begin
                        r3_output_data_reg <= {24'h000000, tmp_read_reg[7:0]};
                    end
                end
                2'b01: begin
                    if(tmp_read_reg[15]) begin
                        r3_output_data_reg <= {24'hffffff, tmp_read_reg[15:8]};
                    end
                    else begin
                        r3_output_data_reg <= {24'h000000, tmp_read_reg[15:8]};
                    end
                end
                2'b10: begin
                    if(tmp_read_reg[23]) begin
                        r3_output_data_reg <= {24'hffffff, tmp_read_reg[23:16]};
                    end
                    else begin
                        r3_output_data_reg <= {24'h000000, tmp_read_reg[23:16]};
                    end
                end
                2'b11: begin
                    if(tmp_read_reg[31]) begin
                        r3_output_data_reg <= {24'hffffff, tmp_read_reg[31:24]};
                    end
                    else begin
                        r3_output_data_reg <= {24'h000000, tmp_read_reg[31:24]};
                    end
                end
                default: begin
                    r3_output_data_reg <= 32'h0000000000;
                end
            endcase
        end
        lw: begin
            stall_reg <= 1'b1;
            write_base <= 1'b0;
            write_ext <= 1'b0;
            write_uart <= 1'b0;
            byte_reg <= 4'b0000;
            r0_instruction_reg <= NOP;
            data_write <= 32'b0;

            casez(addr)
                32'b1000000000????????????????????00: begin
                    read_uart <= 1'b0;
                    r3_output_data_reg <= base_ram_data;
                end
                32'b1000000001????????????????????00: begin
                    read_uart <= 1'b0;
                    r3_output_data_reg <= ext_ram_data;
                end
                32'b00010000000000000000000000000000: begin
                    read_uart <= 1'b1;
                    r3_output_data_reg <= base_ram_data;
                end
                32'b00010000000000000000000000000100: begin
                    read_uart <= 1'b0;
                    r3_output_data_reg <= uart_reg;
                end
                default: begin
                    read_uart <= 1'b0;
                    r3_output_data_reg <= 32'h0000000000;
                end
            endcase
        end
        sb: begin
            stall_reg <= 1'b1;
            write_base <= 1'b0;
            write_ext <= 1'b0;
            write_uart <= 1'b0;
            read_uart <= 1'b0;
            r3_output_data_reg <= r3_ans;
            r0_instruction_reg <= NOP;

            casez(addr)
                32'b1000000000??????????????????????: begin
                    write_base <= 1'b1;
                    write_ext <= 1'b0;
                    write_uart <= 1'b0;
                end
                32'b1000000001??????????????????????: begin
                    write_base <= 1'b0;
                    write_ext <= 1'b1;
                    write_uart <= 1'b0;
                end
                32'b000100000000000000000000000000??: begin
                    write_base <= 1'b0;
                    write_ext <= 1'b0;
                    write_uart <= 1'b1;
                end
                default: begin
                    write_base <= 1'b0;
                    write_ext <= 1'b0;
                    write_uart <= 1'b0;
                end
            endcase

            case(addr[1:0])
                2'b00: begin
                    byte_reg <= 4'b1110;
                    data_write <= {24'h000000, r3_data[7:0]};
                end
                2'b01: begin
                    byte_reg <= 4'b1101;
                    data_write <= {16'h0000, r3_data[7:0], 8'h00};
                end
                2'b10: begin
                    byte_reg <= 4'b1011;
                    data_write <= {8'h00, r3_data[7:0], 16'h0000};
                end
                2'b11: begin
                    byte_reg <= 4'b0111;
                    data_write <= {r3_data[7:0], 24'h000000};
                end
                default: begin
                    byte_reg <= 4'b1111;
                    data_write <= 32'h0000000000;
                end
            endcase
        end
        sw: begin
            stall_reg <= 1'b1;
            read_uart <= 1'b0;
            data_write <= r3_data;
            byte_reg <= 4'b0000;
            r3_output_data_reg <= r3_ans;
            r0_instruction_reg <= NOP;

            casez(addr)
                32'b1000000000????????????????????00: begin
                    write_base <= 1'b1;
                    write_ext <= 1'b0;
                    write_uart <= 1'b0;
                end
                32'b1000000001????????????????????00: begin
                    write_base <= 1'b0;
                    write_ext <= 1'b1;
                    write_uart <= 1'b0;
                end
                default: begin
                    write_base <= 1'b0;
                    write_ext <= 1'b0;
                    write_uart <= 1'b0;
                end
            endcase
        end
        default: begin
            stall_reg <= 1'b0;
            write_base <= 1'b0;
            write_ext <= 1'b0;
            write_uart <= 1'b0;
            read_uart <= 1'b0;
            byte_reg <= 4'b0000;
            data_write <= 32'b0;

            casez(addr)
                32'b1000000000??????????????????????: begin
                    r0_instruction_reg <= base_ram_data;
                end
                32'b1000000001??????????????????????: begin
                    r0_instruction_reg <= ext_ram_data;
                end
                default: begin
                    r0_instruction_reg <= NOP;
                end
            endcase
            //jal jalr
            if(opcode == 7'b1101111 || opcode == 7'b1100111) begin
                r3_output_data_reg <= r3_pc + 4;
            end
            else begin
                r3_output_data_reg <= r3_ans;
            end
        end
    endcase
end

always@(posedge clk or posedge reset_btn) begin
    if(reset_btn) begin
        done <= 1'b0;
    end
    else begin
        if(done) begin
            done <= 1'b0;
        end
        else if(stall_reg) begin
            done <= 1'b1;
        end
    end
end

endmodule
