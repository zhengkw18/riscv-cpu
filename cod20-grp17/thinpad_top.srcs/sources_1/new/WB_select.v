`default_nettype none

module WB_select(
           input wire[31:0] instruction,
           input wire[31:0] ans,
           input wire[31:0] ALU_ans,
           input wire read_offset,

           output wire[31:0] WB_data
       );

reg[31:0] ans_offset;
always@* begin
    case(ALU_ans[1:0])
        2'b00: begin
            if(ans[7]) begin
                ans_offset <= {24'hffffff, ans[7:0]};
            end
            else begin
                ans_offset <= {24'h000000, ans[7:0]};
            end
        end
        2'b01: begin
            if(ans[15]) begin
                ans_offset <= {24'hffffff, ans[15:8]};
            end
            else begin
                ans_offset <= {24'h000000, ans[15:8]};
            end
        end
        2'b10: begin
            if(ans[23]) begin
                ans_offset <= {24'hffffff, ans[23:16]};
            end
            else begin
                ans_offset <= {24'h000000, ans[23:16]};
            end
        end
        2'b11: begin
            if(ans[31]) begin
                ans_offset <= {24'hffffff, ans[31:24]};
            end
            else begin
                ans_offset <= {24'h000000, ans[31:24]};
            end
        end
        default: begin
            ans_offset <= 32'h0000000000;
        end
    endcase
end

wire[6:0] opcode;
assign opcode = instruction[6:0];
localparam
    load_code = 7'b0000011,
    store_code = 7'b0100011;

assign WB_data = (opcode == load_code && read_offset)? ans_offset: ans;

endmodule
