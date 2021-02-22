`default_nettype none

module SRAM(
            input wire clk,           //时钟输入

            input wire clock_btn,         //BTN5手动时钟按钮�???关，带消抖电路，按下时为1
            input wire reset_btn,         //BTN6手动复位按钮�???关，带消抖电路，按下时为1

            //CPLD串口控制器信�???
            output wire uart_rdn,         //读串口信号，低有�???
            output wire uart_wrn,         //写串口信号，低有�???
            input wire uart_dataready,    //串口数据准备�???
            input wire uart_tbre,         //发�?�数据标�???
            input wire uart_tsre,         //数据发�?�完毕标�???

            //BaseRAM信号
            inout wire[31:0] base_ram_data,  //BaseRAM数据，低8位与CPLD串口控制器共�???
            output wire[19:0] base_ram_addr, //BaseRAM地址
            output wire[3:0] base_ram_be_n,  //BaseRAM字节使能，低有效。如果不使用字节使能，请保持�???0
            output wire base_ram_ce_n,       //BaseRAM片�?�，低有�???
            output wire base_ram_oe_n,       //BaseRAM读使能，低有�???
            output wire base_ram_we_n,       //BaseRAM写使能，低有�???

            //ExtRAM信号
            inout wire[31:0] ext_ram_data,  //ExtRAM数据
            output wire[19:0] ext_ram_addr, //ExtRAM地址
            output wire[3:0] ext_ram_be_n,  //ExtRAM字节使能，低有效。如果不使用字节使能，请保持�???0
            output wire ext_ram_ce_n,       //ExtRAM片�?�，低有�???
            output wire ext_ram_oe_n,       //ExtRAM读使能，低有�???
            output wire ext_ram_we_n,       //ExtRAM写使能，低有�???
            
            input wire write_base,
            input wire write_ext,
            input wire write_uart,
            input wire read_uart,
            input wire from_base,
            input wire from_ext,

            input wire[3:0] byte,
            input wire[19:0] addr,
            input wire[31:0] data_write,

            output wire[31:0] SRAM_output
       );

assign base_ram_addr = addr;
assign base_ram_be_n = byte;
assign ext_ram_addr = addr;
assign ext_ram_be_n = byte;

assign base_ram_ce_n = read_uart || write_uart;
assign base_ram_oe_n = read_uart || write_uart;
assign ext_ram_ce_n = 1'b0;
assign ext_ram_oe_n = 1'b0;

assign base_ram_data = (write_base || write_uart)? data_write: 32'bz;
assign base_ram_we_n = (write_base)? clk: 1'b1;
assign ext_ram_data = (write_ext)? data_write: 32'bz;
assign ext_ram_we_n = (write_ext)? clk: 1'b1;

assign uart_rdn = ~read_uart;
assign uart_wrn = ~write_uart;

wire[31:0] base_ram_output;
wire[31:0] ext_ram_output;
wire[31:0] uart_reg_output;

assign base_ram_output = (write_base || write_uart)? 32'b0: base_ram_data; 
assign ext_ram_output = (write_ext)? 32'b0: ext_ram_data; 

reg uart_state;
localparam
    wait_tbre = 0,
    wait_tsre = 1;

wire uart_can_write;
assign uart_can_write = (uart_state == wait_tsre) && uart_tsre;

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

assign uart_reg_output = {16'h0000, 2'b00, uart_can_write, 4'b0000, uart_dataready, 8'h00};

assign SRAM_output = (from_base)? base_ram_data: (from_ext)? ext_ram_data: uart_reg_output;

endmodule
