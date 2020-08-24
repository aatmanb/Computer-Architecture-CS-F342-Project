`timescale 1ns/1ns

module IF (
    input           clk,rst,sel,
    input   [31:0]  Shift_address,
    output  [31:0]  Instruction,
    output  [31:0]  PC_out);
    wire [31:0] PC1,PC2;
    wire Cout,Cin;

    inst_mem mem (Instruction,clk,rst,PC2);
    assign PC_out = PC2 + 32'd1;
    mux2_1 mux (PC_out, Shift_address,sel,PC1);
    D_32 d1(PC1,PC2,clk,rst);
endmodule

module inst_mem(
    output reg [31:0] instruction,
    input clk,
    input rst,
    input [31:0] PC );
    reg [31:0] instMem [0:6];
    integer i;
    initial
    $readmemb("instructions.txt", instMem);
    always@(*)
    begin
    instruction <= instMem[PC];
    end
endmodule

module IF_ID (
    input   [31:0]  in1,in2,
    input           clk,rst,
    output  [31:0]  out1,out2);

    D_32 d1(in1,out1,clk,rst);
    D_32 d2(in2,out2,clk,rst);
endmodule

module ID (
    input       [31:0]  in1,WriteData,
    input               clk,rst,RegWrite,
    input       [4:0]   Destination,
    output      [31:0]  Data1,Data2,
    output  reg [31:0]  extend,
    output      [4:0]   Rd,Rs,Rt);//,Rt);

    always @ (posedge clk)
    begin
        extend[15:0] = in1[15:0];
        extend[31:16] = {16{in1[15]}};
        //extend = $signed(in1[15:0]);
    end
    //assign Rt = in1[20:16];
    assign Rd = in1[15:11];
	assign Rs = in1[25:21];
	assign Rt = in1[20:16];
    RegFile File (clk,rst,RegWrite,in1[25:21],in1[20:16],Destination,WriteData,Data1,Data2);
endmodule

module control( Instruction, WB,MEM,EX,rst);
    input [31:0] Instruction;
    input        rst;
    output       MEM,WB;
    output [2:0] EX;
//    output [1:0] WB;

    reg     [1:0]   ALUOp;
    reg             ALUSrc,Branch,RegWrite;//,Mem2Reg,MemWrite,MemRead,RegDst


    always @(*) begin
        if (rst) begin
            ALUOp = 2'bzz;
            ALUSrc = 1'bz;
            RegWrite = 1'b0;

        end
        else begin
            case(Instruction[31:26])
            // R-Type
                6'b000000: begin
                    ALUOp = 2'b10;
                    ALUSrc = 0;
                    Branch = 0;
                    RegWrite = 1;
                end
            // Conditional Branch
                6'b000100: begin
                    ALUOp = 2'b01;
                    ALUSrc = 0;
                    //RegDst = 1'bx;
                    Branch = 1;
                    RegWrite = 0;
                end
            endcase
        end
    end

    assign WB = {RegWrite};//,Mem2Reg};
    assign MEM = {Branch};//,MemRead,MemWrite};
    assign EX = {ALUOp,ALUSrc};//,RegDst};

endmodule

module RegFile(
    input           clk,reset,RegWrite,
    input   [4:0]   ReadReg1,ReadReg2,WriteReg,
    input   [31:0]  WriteData,
    output  [31:0]  ReadData1,ReadData2);

    wire [31:0] r[31:0];
    wire regclk[31:0];
    wire [31:0]enable,out1,out2,out3,out4,out5,out6,out7,out8,out9,out10,out11,out12,out13,out14,out15,out16,out17,out18,out19,out110;
    genvar i,j;

    decoder d(WriteReg,enable);
    generate
    for(j=0;j<32;j=j+1) begin
    and a1(regclk[j],clk,RegWrite,enable[j]);
    //and a1(regclk[j],clk,1,1);
    end
    endgenerate

    generate
    for(i=0;i<32;i=i+1) begin
    dff_reg f(regclk[i],reset,WriteData,r[i]);
    end
    endgenerate

    mux m1(r[0],r[1],r[2],r[3],ReadReg1[1:0],out1);
    mux m2(r[4],r[5],r[6],r[7],ReadReg1[1:0],out2);
    mux m3(r[8],r[9],r[10],r[11],ReadReg1[1:0],out3);
    mux m4(r[12],r[13],r[14],r[15],ReadReg1[1:0],out4);
    mux m5(r[16],r[17],r[18],r[19],ReadReg1[1:0],out5);
    mux m6(r[20],r[21],r[22],r[23],ReadReg1[1:0],out6);
    mux m7(r[24],r[25],r[26],r[27],ReadReg1[1:0],out7);
    mux m8(r[28],r[29],r[30],r[31],ReadReg1[1:0],out8);
    mux m9(out1,out2,out3,out4,ReadReg1[3:2],out9);
    mux m10(out5,out6,out7,out8,ReadReg1[3:2],out10);
    mux2_1 m11(out9,out10,ReadReg1[4],ReadData1);


    mux m12(r[0],r[1],r[2],r[3],ReadReg2[1:0],out11);
    mux m13(r[4],r[5],r[6],r[7],ReadReg2[1:0],out12);
    mux m14(r[8],r[9],r[10],r[11],ReadReg2[1:0],out13);
    mux m15(r[12],r[13],r[14],r[15],ReadReg2[1:0],out14);
    mux m16(r[16],r[17],r[18],r[19],ReadReg2[1:0],out15);
    mux m17(r[20],r[21],r[22],r[23],ReadReg2[1:0],out16);
    mux m18(r[24],r[25],r[26],r[27],ReadReg2[1:0],out17);
    mux m19(r[28],r[29],r[30],r[31],ReadReg2[1:0],out18);
    mux m20(out11,out12,out13,out14,ReadReg2[3:2],out19);
    mux m21(out15,out16,out17,out18,ReadReg2[3:2],out110);
    mux2_1 m22(out19,out110,ReadReg2[4],ReadData2);
endmodule

module ID_EX (
    input   [31:0]  in1, in2, in3, in4,
    input   [4:0]   in5,in6,in7, //in6,
//    input   [1:0]   wb_in,
//    input   [2:0]   mem_in,
    input   [2:0]   ex_in,
    input           clk,rst,mem_in,wb_in,
    output  [4:0]   Rd,Rs,Rt,//Rt,
    output  [31:0]  A,B,Address,extend,
    output          mem_out,wb_out,ALUSrc,
    output  [1:0]   ALUOp);

    D_32 d1(in1,A,clk,rst);
    D_32 d2(in2,B,clk,rst);
    D_32 d3(in3,Address,clk,rst);
    D_32 d4(in4,extend,clk,rst);
    D_5 d5(in5,Rd,clk,rst);
	D_5 d6(in6,Rs,clk,rst);
	D_5 d7(in7,Rt,clk,rst);

    wire RegWrite,branch; //,MemRead,MemWrite,Mem2Reg;

    d_ff     ff1     (.clk(clk),.rst(rst),.d(wb_in),.q(RegWrite));
    d_ff     ff3     (.clk(clk),.rst(rst),.d(mem_in),.q(branch));
    d_ff     ff6     (.clk(clk),.rst(rst),.d(ex_in[0]),.q(ALUSrc));
    d_ff     ff8     (.clk(clk),.rst(rst),.d(ex_in[1]),.q(ALUOp[0]));
    d_ff     ff9     (.clk(clk),.rst(rst),.d(ex_in[2]),.q(ALUOp[1]));

    assign mem_out = {branch};
    assign wb_out = {RegWrite};
endmodule

module EX(
    input   [31:0]  pc,extended,reg1,reg2,data,
    input           ALUSrc, branch,//RegDst,
    input   [1:0]   ALUOp,
    input rst,
	input A,B,
    output  [31:0]  pc_in,alu_result,
    output          zero, PCSrc);

    wire [31:0] ALUIn1,ALUIn2,ALUIn2_temp;
    wire op0,op1,op2;

    bit32ADD        pc_add      (.in1(pc),.in2(extended<<2),.out(pc_in));
    mux2_1   mux_alu     (.in1(ALUIn2_temp),.in2(extended),.sel(ALUSrc),.out(ALUIn2));
	mux2_1   mux_alu_2     (.in1(reg2),.in2(data),.sel(B),.out(ALUIn2_temp));
	mux2_1   mux_alu_1     (.in1(reg1),.in2(data),.sel(A),.out(ALUIn1));

    MyALU           alu         (.in1(ALUIn1),.rst(rst),.in2(ALUIn2),.op({op0|op2,op1}),.alu_result(alu_result),.zero(zero));
    ALU_ctrl        ctrl        (.func(extended[5:0]),.alu_op0(ALUOp[0]),.alu_op1(ALUOp[1]),.op({op2,op1,op0}));
	and     g1          (PCSrc,branch,zero);
endmodule

module MyALU(
    input [31:0] in1,in2,
    input [1:0] op,
    input rst,
    output [31:0] alu_result,
    output zero);

    wire [31:0] and_wire, or_wire, add_wire, sub_wire;
    reg zero_reg;

    bit32AND  and_op    (.in1(in1),.in2(in2),.out(and_wire));
    bit32OR   or_op     (.in1(in1),.in2(in2),.out(or_wire));
    bit32ADD  add_op    (.in1(in1),.in2(in2),.out(add_wire));
    bit32SUB  sub_op    (.in1(in1),.in2(in2),.out(sub_wire));

    bit32_4to1mux mux (.in1(and_wire),.in2(add_wire),.in3(or_wire),.in4(sub_wire),.sel(op),.out(alu_result));

    always @(*) begin
        if (alu_result==32'b0)
            zero_reg = 1;
        else if(rst==1)
            zero_reg=0;
        else
            zero_reg = 0;
    end
    assign zero = zero_reg;
endmodule

module ALU_ctrl (func,alu_op0,alu_op1,op);

    input alu_op0,alu_op1;
    input [5:0] func;
    output [2:0] op;

    assign op[0] = (~alu_op1 & alu_op0) | (alu_op1 & func[1]);
    assign op[1] = (~func[2]) | (~alu_op1);
    assign op[2] = (func[1] & alu_op1) | alu_op0;
endmodule

module EX_MEM(
    input   [31:0]  pc_in,alu_result_in,//mem_write_data_in,
    input   [4:0]   dstreg_in,
    input           zero_in,clk,rst,wb_in,mem_in,
    output  [31:0]  pc_out,alu_result_out,//mem_write_data_out,
    output  [4:0]   dstreg_out,
    output          zero_out,mem_out,wb_out);//MemRead,MemWrite,

    wire RegWrite,Branch;//,Mem2Reg;

    D_32   pc_reg              (.in(pc_in),.clk(clk),.rst(rst),.out(pc_out));
    D_32   alu_result_reg      (.in(alu_result_in),.clk(clk),.rst(rst),.out(alu_result_out));
    D_5    dstreg_reg          (.in(dstreg_in),.clk(clk),.rst(rst),.out(dstreg_out));
    d_ff        zero_reg            (.d(zero_in),.clk(clk),.rst(rst),.q(zero_out));
    d_ff        RegWrite_reg        (.d(wb_in),.clk(clk),.rst(rst),.q(RegWrite));
    //d_ff        Mem2Reg_reg         (.d(wb_in[1]),.clk(clk),.rst(rst),.q(Mem2Reg));
    d_ff        branch_reg          (.d(mem_in),.clk(clk),.rst(rst),.q(mem_out));

    assign wb_out = {RegWrite};//,Mem2Reg};
	//assign mem_out = {Branch};
endmodule

module MEM(
    //input   [31:0]  addr,mem_write_data,
    input           branch,zero,//,mem_read,mem_write,initializeMemory,
    //output  [31:0]  mem_read_data,
    output          PCSrc);

    and     g1          (PCSrc,branch,zero);

    //memory  data_mem    (.address(addr),.dataIn(mem_write_data),.readEnable(mem_read),.writeEnable(mem_write),
    //                     .initializeMemory(initializeMemory),.printOutput(1'b0),.memOut(mem_read_data));
endmodule

module MEM_WB(
    input   [31:0]  alu_result_in,//mem_read_data_in,
    input   [4:0]   dstreg_in,
    input           clk,rst,wb_in,
    output  [31:0]  alu_result_out,//mem_read_data_out,
    output  [4:0]   dstreg_out,
    output          RegWrite);

    //reg_32bit   mem_data    (.d(mem_read_data_in),.clk(clk),.rst(rst),.q(mem_read_data_out));
    D_32   alu_result  (.in(alu_result_in),.clk(clk),.rst(rst),.out(alu_result_out));
    D_5    dstreg_reg  (.in(dstreg_in),.clk(clk),.rst(rst),.out(dstreg_out));
    d_ff        wb_reg      (.d(wb_in),.clk(clk),.rst(rst),.q(RegWrite));
endmodule

module WB(
    input   [31:0]  alu_result,//mem_read_data,
    //input           mem2reg,
    output  [31:0]  write_data);

    //bit32_2to1mux   mux     (.in1(mem_read_data),.in2(alu_result),.sel(mem2reg),.out(write_data));
    assign write_data = alu_result;
endmodule







module mux2to1(out,sel,in1,in2);
    input in1,in2,sel;
    output out;

    wire not_sel,a1,a2;

    not (not_sel,sel);
    and (a1,sel,in2);
    and (a2,not_sel,in1);
    or  (out,a1,a2);
endmodule

module bit5_2to1mux(out,sel,in1,in2);
    input [4:0] in1,in2;
    output [4:0] out;
    input sel;

    genvar j;
    //this is the variable that is be used in the generate block
    generate
        for (j=0; j<5;j=j+1) begin: mux_loop
        //mux_loop is the name of the loop
            mux2to1 m1(out[j],sel,in1[j],in2[j]);
        //mux2to1 is instantiated every time it is called
        end
    endgenerate
endmodule

module bit8_2to1mux(out,sel,in1,in2);
    input [7:0] in1,in2;
    output [7:0] out;
    input sel;

    genvar j;
    //this is the variable that is be used in the generate block
    generate
        for (j=0; j<8;j=j+1) begin: mux_loop
        //mux_loop is the name of the loop
            mux2to1 m1(out[j],sel,in1[j],in2[j]);
        //mux2to1 is instantiated every time it is called
        end
    endgenerate
endmodule

module bit32_2to1mux(out,sel,in1,in2);
    input [31:0] in1,in2;
    input sel;
    output [31:0] out;

    genvar j;
    generate
        for (j=0;j<4;j=j+1) begin
            bit8_2to1mux m1(out[8*(j+1)-1:8*j],sel,in1[8*(j+1)-1:8*j],in2[8*(j+1)-1:8*j]);
        end
    endgenerate
endmodule

module bit32_4to1mux(
    output [31:0] out,
    input [31:0] in1,in2,in3,in4,
    input [1:0] sel);

    wire [31:0] out1,out2;

    bit32_2to1mux mux1(.out(out1),.sel(sel[0]),.in1(in1),.in2(in2));
    bit32_2to1mux mux2(.out(out2),.sel(sel[0]),.in1(in3),.in2(in4));
    bit32_2to1mux mux3(.out(out),.sel(sel[1]),.in1(out1),.in2(out2));
endmodule

module mux2_1(in1,in2,sel,out);
    input [31:0] in1,in2;
    input sel;
    output reg [31:0]out;
    always @*
    begin
    case (sel)
    1'b0: out<=in1;
    1'b1: out<=in2;
    endcase
    end
endmodule


module mux(in1,in2,in3,in4,sel,Out);
    input [31:0] in1,in2,in3,in4;
    input [1:0] sel;
    output reg [31:0] Out;
    always @ *
    begin
    case(sel)
    2'b00: Out<=in1;
    2'b01: Out<=in2;
    2'b10: Out<=in3;
    2'b11: Out<=in4;
    endcase
    end
 endmodule

module bit32AND (
    output [31:0] out,
    input [31:0] in1,in2);

    assign {out}=in1 & in2;
endmodule

module bit32OR (
    output [31:0] out,
    input [31:0] in1,in2);

    assign {out}=in1 | in2;
endmodule

module bit32ADD (
    output [31:0] out,
    input [31:0] in1,in2);

    assign {out}=in1 + in2;
endmodule

module bit32SUB (
    output [31:0] out,
    input [31:0] in1,in2);

    assign {out}=in1 + (~in2) + 1;
endmodule

// D flip flop with asynchronous reset
module d_ff(
    input d,clk,rst,
    output reg q);

    always @(posedge clk or posedge rst) begin
        if(rst)
            q <= 1'b0;
        else
            q <= d;
    end
endmodule
/*
module reg_5bit(
    input [4:0] d,
    input clk,rst,
    output [4:0] q);

    genvar i;
    generate
        for(i=0;i<5;i=i+1) begin: flip_flop
            d_ff ff(.q(q[i]),.clk(clk),.rst(rst),.d(d[i]));
        end
    endgenerate
endmodule
*/
/*
module reg_32bit(
    input [31:0] d,
    input clk, rst,
    output [31:0] q);

    genvar i;
    generate
        for(i=0;i<32;i=i+1) begin: flip_flop
            d_ff ff(.q(q[i]),.clk(clk),.rst(rst),.d(d[i]));
        end
    endgenerate
endmodule
*/
module D_32(in,out,clk,rst);
    input[31:0] in;
    input clk,rst;
    output reg [31:0] out;
    always @ (posedge clk or posedge rst)
    begin
    if(rst==1)
    out<=0;
    else
    out<=in;
    end
endmodule
/*
module dff(clk,rst,D,Q);
    input [31:0] D;
    input clk,rst;
    output reg [31:0]Q;
    always @(posedge clk , posedge rst)
    begin
    if(rst==1) begin
    Q<=32'h0;
    end
    else
    Q<=D;
    end
endmodule
*/

module dff_reg(clk,rst,D,Q);
    input [31:0] D;
    input clk,rst;
    output reg [31:0]Q;
    always @(negedge clk or posedge rst)
    begin
    if(rst==1) begin
    Q<=32'd0;
    end
    else
    Q<=D;
    end
endmodule

module D_5(in,out,clk,rst);
    input[4:0] in;
    input clk,rst;
    output reg [4:0] out;
    always @ (posedge clk or posedge rst)
    begin
    if(rst==1)
    out<=0;
    else
    out<=in;
    end
endmodule

module decoder(in,out);
    input [4:0] in;
    output reg [31:0] out;
    always @ *
    begin
    case(in)
    5'b00000: out<=32'b00000001;
    5'b00001: out<=32'h00000002;
    5'b00010: out<=32'h00000004;
    5'b00011: out<=32'h00000008;
    5'b00100: out<=32'h00000010;
    5'b00101: out<=32'h00000020;
    5'b00110: out<=32'h00000040;
    5'b00111: out<=32'h00000080;
    5'b01000: out<=32'h00000100;
    5'b01001: out<=32'h00000200;
    5'b01010: out<=32'h00000400;
    5'b01011: out<=32'h00000800;
    5'b01100: out<=32'h00001000;
    5'b01101: out<=32'h00002000;
    5'b01110: out<=32'h00004000;
    5'b01111: out<=32'h00008000;
    5'b10000: out<=32'h00010000;
    5'b10001: out<=32'h00020000;
    5'b10010: out<=32'h00040000;
    5'b10011: out<=32'h00080000;
    5'b10100: out<=32'h00100000;
    5'b10101: out<=32'h00200000;
    5'b10110: out<=32'h00400000;
    5'b10111: out<=32'h00800000;
    5'b11000: out<=32'h01000000;
    5'b11001: out<=32'h02000000;
    5'b11010: out<=32'h04000000;
    5'b11011: out<=32'h08000000;
    5'b11100: out<=32'h10000000;
    5'b11101: out<=32'h20000000;
    5'b11110: out<=32'h40000000;
    5'b11111: out<=32'h80000000;
    endcase
    end
endmodule

module hazard(input [4:0] Rs,Rt,Rd,
	input Rst,
	output reg A,B);

	always @(*) begin
	if(Rst==1) begin
	A=0;
	B=0;
	end
	else if(Rs==Rd) begin
	A=1;
	B=0;
	end
	else if(Rt==Rd) begin
	A=0;
	B=1;
	end
	else begin
	A=0;
	B=0;
	end
	end
endmodule

module neg_d_ff(
    input d,clk,rst,
    output reg q);

    always @(negedge clk or posedge rst) begin
        if(rst)
            q <= 1'b0;
        else
            q <= d;
    end
endmodule

