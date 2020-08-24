`timescale 1ns/1ns
`include "src.v"

module tb();
	reg clk,rst;

	top tp(.clk(clk),.rst(rst));
	initial begin
	clk=0;
	rst=0;
	#5 rst=1;
	#5 rst=0;

	#110 $finish;
	end
	always #5 clk=~clk;
	initial begin
	$dumpfile("MIPS.vcd");
	$dumpvars;

	end
endmodule

module top (
    input clk,rst);

    wire            flush;
	wire            PCSrc,RegWrite,ALUSrc,zero_EXOut,zero_MEMIn,branch;//RegDst,MemRead,MemWrite,Mem2Reg;
    wire    [1:0]   ALUOp;
    wire    [2:0]   EX_ctrl;
    wire            MEM_ctrl,MEM_ctrl1;
    wire            WB_ctrl,WB_ctrl1,WB_ctrl2;
    wire    [31:0]  Shift_address,WriteData;//mem_write_data_MEMIn;
    wire    [31:0]  Rs_IDOut,Rt_IDOut,Rs_EXIn,Rt_EXIn;
    wire    [31:0]  extended_IDOut,extended_EXIn;
    wire    [31:0]  instr_IFOut,instr_IDIn;
    wire    [31:0]  PC_IFOut,PC_IDIn,PC_EXIn,PC_EXOut;//,PC_IDOut;
    wire    [31:0]  ALUResult_EXOut,ALUResult_MEMIn,ALUResult_WBIn;
	wire 			a,b;
    wire    [4:0]   WriteReg,Rs_out,Rt_out,Rd_addr_IDOut,Rs_Addr_ID_out,Rt_Addr_ID_out,Rd_addr_EXIn,Rd_addr_MEMIn;//,Rt_addr_EXIn,Rt_addr_IDOut;
    //wire    [4:0]   DstReg_EXOut,DstReg_MEMIn,DstReg_WBIn;//,DstReg_MEMOut;

    IF      fetch   (.clk(clk),.rst(rst),.sel(PCSrc),.Shift_address(Shift_address),.PC_out(PC_IFOut),.Instruction(instr_IFOut));

    IF_ID   if_id   (.clk(clk),.rst(flush),.in1(PC_IFOut),.in2(instr_IFOut),.out1(PC_IDIn),.out2(instr_IDIn));

    ID      decode  (.clk(clk),.rst(rst),.in1(instr_IDIn),.WriteData(WriteData),.RegWrite(RegWrite),
                     .Destination(WriteReg),.Data1(Rs_IDOut),.Data2(Rt_IDOut),.extend(extended_IDOut),
                     .Rd(Rd_addr_IDOut),.Rs(Rs_out),.Rt(Rt_out));

    control ctrl    (.rst(rst),.Instruction(instr_IDIn),.EX(EX_ctrl),.MEM(MEM_ctrl),.WB(WB_ctrl));

    ID_EX   id_ex   (.clk(clk),.rst(flush),.in1(Rs_IDOut),.in2(Rt_IDOut),.in3(PC_IDIn),.in4(extended_IDOut),
                     .in5(Rd_addr_IDOut),.in6(Rs_out),.in7(Rt_out),.ex_in(EX_ctrl),.mem_in(MEM_ctrl),.wb_in(WB_ctrl),
                     .Rd(Rd_addr_EXIn),.A(Rs_EXIn),.B(Rt_EXIn),.Address(PC_EXIn),.extend(extended_EXIn),
                     .wb_out(WB_ctrl1),.mem_out(MEM_ctrl1),.ALUSrc(ALUSrc),.ALUOp(ALUOp),.Rs(Rs_Addr_ID_out),.Rt(Rt_Addr_ID_out));

    EX      execute (.pc(PC_EXIn),.rst(rst),.extended(extended_EXIn),.reg1(Rs_EXIn),.reg2(Rt_EXIn),.ALUSrc(ALUSrc),.ALUOp(ALUOp),
                     .pc_in(PC_EXOut),.alu_result(ALUResult_EXOut),.zero(zero_EXOut),.data(ALUResult_MEMIn),.A(a),.B(b));

    EX_MEM  ex_mem  (.clk(clk),.rst(rst),.pc_in(PC_EXOut),.alu_result_in(ALUResult_EXOut),.dstreg_in(Rd_addr_EXIn),
                     .zero_in(zero_EXOut),.wb_in(WB_ctrl1),.mem_in(MEM_ctrl1),.pc_out(Shift_address),
                     .alu_result_out(ALUResult_MEMIn),.dstreg_out(Rd_addr_MEMIn),.zero_out(zero_MEMIn),.mem_out(branch),
                     .wb_out(WB_ctrl2));

    MEM     mem     (.PCSrc(PCSrc),.branch(branch),.zero(zero_MEMIn));

    MEM_WB  mem_wb  (.clk(clk),.rst(rst),.alu_result_in(ALUResult_MEMIn),.dstreg_in(Rd_addr_MEMIn),.wb_in(WB_ctrl2),
                     .alu_result_out(ALUResult_WBIn),.dstreg_out(WriteReg),.RegWrite(RegWrite));

    WB      WriteBack (.alu_result(ALUResult_WBIn),.write_data(WriteData));

	hazard hz (.Rs(Rs_Addr_ID_out),.Rt(Rt_Addr_ID_out),.Rd(Rd_addr_MEMIn),.Rst(rst),.A(a),.B(b));

	or g1 (flush, rst,branch);
endmodule
