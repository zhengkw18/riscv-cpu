`default_nettype none

module thinpad_top(
           input wire clk_50M,           //50MHz 时钟输入
           input wire clk_11M0592,       //11.0592MHz 时钟输入（备用，可不用）

           input wire clock_btn,         //BTN5手动时钟按钮�????????关，带消抖电路，按下时为1
           input wire reset_btn,         //BTN6手动复位按钮�????????关，带消抖电路，按下时为1

           input  wire[3:0]  touch_btn,  //BTN1~BTN4，按钮开关，按下时为1
           input  wire[31:0] dip_sw,     //32位拨码开关，拨到“ON”时�????????1
           output wire[15:0] leds,       //16位LED，输出时1点亮
           output wire[7:0]  dpy0,       //数码管低位信号，包括小数点，输出1点亮
           output wire[7:0]  dpy1,       //数码管高位信号，包括小数点，输出1点亮

           //CPLD串口控制器信�????????
           output wire uart_rdn,         //读串口信号，低有�????????
           output wire uart_wrn,         //写串口信号，低有�????????
           input wire uart_dataready,    //串口数据准备�????????
           input wire uart_tbre,         //发�?�数据标�????????
           input wire uart_tsre,         //数据发�?�完毕标�????????

           //BaseRAM信号
           inout wire[31:0] base_ram_data,  //BaseRAM数据，低8位与CPLD串口控制器共�????????
           output wire[19:0] base_ram_addr, //BaseRAM地址
           output wire[3:0] base_ram_be_n,  //BaseRAM字节使能，低有效。如果不使用字节使能，请保持�????????0
           output wire base_ram_ce_n,       //BaseRAM片�?�，低有�????????
           output wire base_ram_oe_n,       //BaseRAM读使能，低有�????????
           output wire base_ram_we_n,       //BaseRAM写使能，低有�????????

           //ExtRAM信号
           inout wire[31:0] ext_ram_data,  //ExtRAM数据
           output wire[19:0] ext_ram_addr, //ExtRAM地址
           output wire[3:0] ext_ram_be_n,  //ExtRAM字节使能，低有效。如果不使用字节使能，请保持�????????0
           output wire ext_ram_ce_n,       //ExtRAM片�?�，低有�????????
           output wire ext_ram_oe_n,       //ExtRAM读使能，低有�????????
           output wire ext_ram_we_n,       //ExtRAM写使能，低有�????????

           //直连串口信号
           output wire txd,  //直连串口发�?�端
           input  wire rxd,  //直连串口接收�????????

           //Flash存储器信号，参�?? JS28F640 芯片手册
           output wire [22:0]flash_a,      //Flash地址，a0仅在8bit模式有效�????????16bit模式无意�????????
           inout  wire [15:0]flash_d,      //Flash数据
           output wire flash_rp_n,         //Flash复位信号，低有效
           output wire flash_vpen,         //Flash写保护信号，低电平时不能擦除、烧�????????
           output wire flash_ce_n,         //Flash片�?�信号，低有�????????
           output wire flash_oe_n,         //Flash读使能信号，低有�????????
           output wire flash_we_n,         //Flash写使能信号，低有�????????
           output wire flash_byte_n,       //Flash 8bit模式选择，低有效。在使用flash�????????16位模式时请设�????????1

           //USB 控制器信号，参�?? SL811 芯片手册
           output wire sl811_a0,
           //inout  wire[7:0] sl811_d,     //USB数据线与网络控制器的dm9k_sd[7:0]共享
           output wire sl811_wr_n,
           output wire sl811_rd_n,
           output wire sl811_cs_n,
           output wire sl811_rst_n,
           output wire sl811_dack_n,
           input  wire sl811_intrq,
           input  wire sl811_drq_n,

           //网络控制器信号，参�?? DM9000A 芯片手册
           output wire dm9k_cmd,
           inout  wire[15:0] dm9k_sd,
           output wire dm9k_iow_n,
           output wire dm9k_ior_n,
           output wire dm9k_cs_n,
           output wire dm9k_pwrst_n,
           input  wire dm9k_int,

           //图像输出信号
           output wire[2:0] video_red,    //红色像素�????????3�????????
           output wire[2:0] video_green,  //绿色像素�????????3�????????
           output wire[1:0] video_blue,   //蓝色像素�????????2�????????
           output wire video_hsync,       //行同步（水平同步）信�????????
           output wire video_vsync,       //场同步（垂直同步）信�????????
           output wire video_clk,         //像素时钟输出
           output wire video_de           //行数据有效信号，用于区分消隐�????????
       );

wire locked, clk_50M_pll, clk_20M;
pll_example clock_gen
            (
                // Clock in ports
                .clk_in1(clk_50M),  // �ⲿʱ������
                // Clock out ports
                .clk_out1(clk_50M_pll), // ʱ�����?????1��Ƶ����IP���ý���������
                .clk_out2(clk_20M), // ʱ�����?????2��Ƶ����IP���ý���������
                // Status and control signals
                .reset(reset_btn), // PLL��λ����
                .locked(locked)    // PLL����ָʾ�����??????"1"��ʾʱ���ȶ���
                // �󼶵�·��λ�ź�Ӧ���������ɣ����£�
            );

// wire reset_of_clk50M;
// assign reset_of_clk50M = ~locked;
reg reset_of_clk50M;
// 异步复位，同步释放，将locked信号转为后级电路的复位reset_of_clk10M
always@(posedge clk_50M_pll or negedge locked) begin
    if(~locked) reset_of_clk50M <= 1'b1;
    else        reset_of_clk50M <= 1'b0;
end

//图像输出演示，分辨率800x600@75Hz，像素时钟为50MHz
wire [11:0] hdata;
wire [11:0] vdata;
// assign video_red = hdata < 266 ? 3'b111 : 0; //红色竖条
// assign video_green = hdata < 532 && hdata >= 266 ? 3'b111 : 0; //绿色竖条
// assign video_blue = hdata >= 532 ? 2'b11 : 0; //蓝色竖条
localparam NOP = 32'b00000000000000000000000000010011; // addi x0,x0,0
localparam START = 32'h80000000;
localparam ADDI = 5'b00100;

reg[31:0] r0_pc;
reg[31:0] r0_pc_copy;

reg[31:0] r1_pc;
reg[31:0] r1_instr;
reg r1_jmp;

reg[31:0] r2_pc;
reg[31:0] r2_instr;
reg[4:0] r2_opcode;
reg[31:0] r2_dataA;
reg[31:0] r2_dataB;
reg[31:0] r2_imm;
reg[31:0] r2_csr_ans;
reg r2_jmp;

reg[31:0] r3_instr;
reg[31:0] r3_instr_copy;
reg[31:0] r3_ans;
reg[31:0] r3_ALU_ans;
reg r3_write_base;
reg r3_write_ext;
reg r3_write_uart;
reg r3_read_uart;
reg r3_from_base;
reg r3_from_ext;
reg[3:0] r3_byte;
reg[19:0] r3_addr;
reg[19:0] r3_addr_copy;
reg[31:0] r3_data_write;
reg r3_stall;
reg r3_read_offset;
reg r3_bram_we_n;
reg r3_ans_en;
reg[31:0] SRAM_ans;

reg[31:0] r4_instr;
reg[31:0] r4_instr_copy;
reg[31:0] r4_ans;
reg[31:0] r4_ans_copy;
reg[31:0] r4_ALU_ans;
reg[31:0] r4_read_offset;

wire[31:0] ALU_ans;
wire ALU_is_jumping;
wire ALU_is_bj;
wire[31:0] ALU_pc_imm;

wire[4:0] DECODER_op_code;

wire CONTROLLER_is_nop;
wire[31:0] CONTROLLER_next_pc;
wire CONTROLLER_jmp;

wire[31:0] FORWARD_A_out;
wire[31:0] FORWARD_B_out;

wire[31:0] IMM_imm;

wire[31:0] REG_A;
wire[31:0] REG_B;

wire[31:0] SRAM_output_data;

wire de_write_base;
wire de_write_ext;
wire de_write_uart;
wire de_read_uart;
wire de_from_base;
wire de_from_ext;
wire[3:0] de_byte;
wire[19:0] de_SRAM_addr;
wire[31:0] de_SRAM_data;
wire de_mem_stall;
wire de_read_offset;
wire de_bram_we_n;
wire de_ans_en;

wire[31:0] CSR_csr_ans;
wire[31:0] CSR_satp;
wire CSR_status;

wire[31:0] WB_data_wire;

wire[16:0] bram_write_addr;
assign bram_write_addr = r3_addr_copy[16:0];
wire[31:0] bram_write_data;
assign bram_write_data = r3_data_write;
wire[3:0] bram_be_n;
assign bram_be_n = ~r3_byte;
wire bram_we_n;
assign bram_we_n = r3_bram_we_n;
wire[18:0] bram_read_addr;
wire[31:0] bram_read_data;
wire bram_oe_n;
assign bram_oe_n = 1'b1;

reg[7:0] vga_data_reg;
always@* begin
    case(bram_read_addr[1:0])
        2'b01: vga_data_reg <= bram_read_data[7:0];
        2'b10: vga_data_reg <= bram_read_data[15:8];
        2'b11: vga_data_reg <= bram_read_data[23:16];
        2'b00: vga_data_reg <= bram_read_data[31:24];
        default: vga_data_reg <= 8'hff;
    endcase
end
assign video_red = vga_data_reg[7:5];
assign video_green = vga_data_reg[4:2];
assign video_blue = vga_data_reg[1:0];
assign video_clk = clk_50M;
vga #(12, 800, 856, 976, 1040, 600, 637, 643, 666, 1, 1) vga800x600at75 (
        .clk(clk_50M),
        .hdata(hdata), //横坐�?????
        .vdata(vdata), //纵坐�?????
        .hsync(video_hsync),
        .vsync(video_vsync),
        .data_enable(video_de),
        .addr_to_read(bram_read_addr)
    );

 blk_mem_gen_0 _blk_mem_gen_0 (
                   .clka(clk_50M_pll),
                   .ena(bram_we_n),
                   .wea(bram_be_n),
                   .addra(bram_write_addr),
                   .dina(bram_write_data),
                   .clkb(clk_50M),
                   .enb(bram_oe_n),
                   .addrb(bram_read_addr[18:2]),
                   .doutb(bram_read_data)
               );

ALU _ALU(
        .ans(ALU_ans),
        .is_jumping(ALU_is_jumping),
        .is_bj(ALU_is_bj),
        .pc_imm(ALU_pc_imm),

        .instr(r2_instr),
        .pc(r2_pc),
        .operator(r2_opcode),
        .dataA(FORWARD_A_out),
        .dataB(FORWARD_B_out),
        .imm(r2_imm),
        .csr_ans(r2_csr_ans)
    );

DECODER _DECODER(
            .op_code(DECODER_op_code),			// ćäť¤čŻç 
            .ins(r1_instr)					// ćäť¤
        );

CONTROLLER _CONTROLLER(
               .clk(clk_50M_pll),
               .reset_btn(reset_of_clk50M),
               .current_pc(r0_pc_copy),
               .r1_pc(r1_pc),
               .r2_pc(r2_pc),
               .r2_jmp(r2_jmp),
               .pc_plus_imm(ALU_pc_imm),
               .ALU_jmp(ALU_is_jumping),
               .stall(r3_stall),

               .is_nop(CONTROLLER_is_nop),
               .next_pc(CONTROLLER_next_pc),
               .jmp(CONTROLLER_jmp)
           );

FORWARD _FORWARD(
            .r3_instr(r3_instr_copy),
            .r4_instr(r4_instr_copy),
            .r3_ans(WB_data_wire),
            .r4_ans(r4_ans_copy),
            .A_in(r2_dataA),
            .B_in(r2_dataB),
            .r2_instr(r2_instr),

            .A_out(FORWARD_A_out),
            .B_out(FORWARD_B_out)
        );

IMM _IMM(
        .instr(r1_instr),

        .imm(IMM_imm)
    );

REG _REG(
        .A(REG_A),
        .B(REG_B),

        .clk(clk_50M_pll),
        .r4_instr(r4_instr),
        .r4_ans(r4_ans),
        .r1_instr(r1_instr)
    );

SRAM _SRAM(
         .clk(clk_50M_pll),

         .clock_btn(clock_btn),
         .reset_btn(reset_of_clk50M),

         .uart_rdn(uart_rdn),
         .uart_wrn(uart_wrn),
         .uart_dataready(uart_dataready),
         .uart_tbre(uart_tbre),
         .uart_tsre(uart_tsre),

         .base_ram_data(base_ram_data),
         .base_ram_addr(base_ram_addr),
         .base_ram_be_n(base_ram_be_n),
         .base_ram_ce_n(base_ram_ce_n),
         .base_ram_oe_n(base_ram_oe_n),
         .base_ram_we_n(base_ram_we_n),

         .ext_ram_data(ext_ram_data),
         .ext_ram_addr(ext_ram_addr),
         .ext_ram_be_n(ext_ram_be_n),
         .ext_ram_ce_n(ext_ram_ce_n),
         .ext_ram_oe_n(ext_ram_oe_n),
         .ext_ram_we_n(ext_ram_we_n),

         .write_base(r3_write_base),
         .write_ext(r3_write_ext),
         .write_uart(r3_write_uart),
         .read_uart(r3_read_uart),
         .from_base(r3_from_base),
         .from_ext(r3_from_ext),

         .byte(r3_byte),
         .addr(r3_addr),
         .data_write(r3_data_write),

         .SRAM_output(SRAM_output_data)
     );

// assign leds = {FORWARD_A_out[7:0], FORWARD_B_out[7:0]};
// assign leds = {SRAM_mem_stall, 12'b0, ALU_is_jumping, CONTROLLER_stall_pc, CONTROLLER_pc_selector};
// assign leds = {r3_b[7:0], SRAM_r3_output_data[7:0]};
assign leds = 16'b0;
assign dpy0 = 8'b0;
assign dpy1 = 8'b0;
wire[31:0] next_pc;
assign next_pc =  r3_stall ? r0_pc_copy : CONTROLLER_next_pc;

SRAM_decoder _SRAM_decoder(
                 .clk(clk_50M_pll),

                 .clock_btn(clock_btn),
                 .reset_btn(reset_of_clk50M),

                 .r2_instruction(r2_instr),
                 .ALU_ans(ALU_ans),
                 .ALU_B(FORWARD_B_out),
                 .r0_pc(next_pc),

                 .SRAM_ans(SRAM_ans),
                 .satp(CSR_satp),
                 .status(CSR_status),

                 .write_base(de_write_base),
                 .write_ext(de_write_ext),
                 .write_uart(de_write_uart),
                 .read_uart(de_read_uart),
                 .from_base(de_from_base),
                 .from_ext(de_from_ext),

                 .byte(de_byte),
                 .SRAM_addr(de_SRAM_addr),
                 .SRAM_data(de_SRAM_data),
                 .mem_stall(de_mem_stall),
                 .read_offset(de_read_offset),

                 .bram_we_n(de_bram_we_n),
                 .r3_ans_en(de_ans_en)
             );

WB_select _WB_select_r3(
              .instruction(r3_instr),
              .ans(r3_ans),
              .ALU_ans(r3_ALU_ans),
              .read_offset(r3_read_offset),

              .WB_data(WB_data_wire)
          );

CSR _CSR(
        .csr_ans(CSR_csr_ans),
        .csr_satp(CSR_satp),
        .csr_status(CSR_status),

        .clk(clk_50M_pll),
        .reset_btn(reset_of_clk50M),
        .r1_instr(r1_instr),
        .r2_instr(r2_instr),
        .r2_csr_ans(r2_csr_ans),
        .A(FORWARD_A_out),
        .pc(r2_pc),
        .stall(r3_stall)
    );

always @(posedge clk_50M_pll or posedge reset_of_clk50M) begin
    if(reset_of_clk50M) begin
        r0_pc <= START;
        r0_pc_copy <= START;
        r1_instr <= NOP;
        r1_jmp <= 1'b0;
        r2_instr <= NOP;
        r2_opcode <= ADDI;
        r2_jmp <= 1'b0;
        r3_instr <= NOP;
        r3_instr_copy <= NOP;
        r4_instr <= NOP;
        r4_instr_copy <= NOP;
        r3_write_base <= 1'b0;
        r3_write_ext <= 1'b0;
        r3_write_uart <= 1'b0;
        r3_read_uart <= 1'b0;
        r3_from_base <= 1'b1;
        r3_from_ext <= 1'b0;
        r3_byte <= 4'b0000;
        r3_addr <= START;
        r3_data_write <= 32'b0;
        r3_stall <= 1'b0;
        r3_bram_we_n <= 1'b0;
        r3_ans_en <= 1'b0;
    end
    else begin
        //r0
        r0_pc <= next_pc;
        r0_pc_copy <= next_pc;

        if(!r3_stall) begin
            //r1
            r1_pc <= r0_pc;
            r1_instr <= CONTROLLER_is_nop ? NOP : SRAM_output_data;
            r1_jmp <= CONTROLLER_is_nop ? 1'b0 : CONTROLLER_jmp;
            //r2
            r2_pc <= r1_pc;
            r2_instr <= CONTROLLER_is_nop ? NOP : r1_instr;
            r2_opcode <= CONTROLLER_is_nop ? ADDI : DECODER_op_code;
            r2_dataA <= REG_A;
            r2_dataB <= REG_B;
            r2_imm <= IMM_imm;
            r2_csr_ans <= CSR_csr_ans;
            r2_jmp <= CONTROLLER_is_nop ? 1'b0 : r1_jmp;
            //r3
            r3_instr <= r2_instr;
            r3_instr_copy <= r2_instr;
            r3_ans <= ALU_ans;
            r3_ALU_ans <= ALU_ans;
            r3_read_offset <= de_read_offset;
            //r4
            r4_instr <= r3_instr;
            r4_instr_copy <= r3_instr;
            r4_ALU_ans <= r3_ALU_ans;
            r4_ans <= WB_data_wire;
            r4_ans_copy <= WB_data_wire;
            r4_read_offset <= r3_read_offset;
        end
        else begin
            if(r3_ans_en) begin
                r3_ans <= SRAM_output_data;
            end
            SRAM_ans <= SRAM_output_data;
        end
        r3_write_base <= de_write_base;
        r3_write_ext <= de_write_ext;
        r3_write_uart <= de_write_uart;
        r3_read_uart <= de_read_uart;
        r3_from_base <= de_from_base;
        r3_from_ext <= de_from_ext;
        r3_byte <= de_byte;
        r3_addr <= de_SRAM_addr;
        r3_addr_copy <= de_SRAM_addr;
        r3_data_write <= de_SRAM_data;
        r3_stall <= de_mem_stall;
        r3_bram_we_n <= de_bram_we_n;
        r3_ans_en <= de_ans_en;
    end
end

endmodule
