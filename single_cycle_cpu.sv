/* ********************************************
 *	COSE222 Lab #3
 *
 *	Module: top design of the single-cycle CPU (single_cycle_cpu.sv)
 *  - Top design of the single-cycle CPU
 *
 *  Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps
`define FF 1    // Flip-flop delay for just better waveform view

module single_cycle_cpu
#(  parameter IMEM_DEPTH = 1024,    // imem depth (default: 1024 entries = 4 KB)
              IMEM_ADDR_WIDTH = 10,
              REG_WIDTH = 64,
              DMEM_DEPTH = 1024,    // dmem depth (default: 1024 entries = 8 KB)
              DMEM_ADDR_WIDTH = 10 )
(
    input           clk,            // System clock
    input           reset_b         // Asychronous negative reset
);

    // Wires for datapath elements
    logic   [IMEM_ADDR_WIDTH-1:0]   imem_addr;
    logic   [31:0]  inst;   // instructions = an output of ????

    logic   [4:0]   rs1, rs2, rd;    // register numbers
    logic   [REG_WIDTH-1:0] rd_din;
    logic           reg_write;
    logic   [REG_WIDTH-1:0] rs1_dout, rs2_dout;

    logic   [REG_WIDTH-1:0] alu_in1, alu_in2;
    logic   [3:0]   alu_control;    // ALU control signal
    logic   [REG_WIDTH-1:0] alu_result;
    logic           alu_zero;

    logic   [DMEM_ADDR_WIDTH-1:0]    dmem_addr;
    logic   [63:0]  dmem_din, dmem_dout;
    logic           mem_read, mem_write;

    // -------------------------------------------------------------------
    /* Main control unit:
     * Main control unit generates control signals for datapath elements
     * The control signals are determined by decoding instructions
     * Generating control signals using opcode = inst[6:0]
     */
    logic   [6:0]   opcode;
    logic           branch, alu_src, mem_to_reg;
    logic   [1:0]   alu_op;
    //logic         mem_read, mem_write, reg_write; // declared above

    // COMPLETE THE MAIN CONTROL UNIT HERE
    // beq 7'b1100011, lw 7'b0000011, sw 7'b0100011, r 7'b0110011

    assign opcode = inst[6:0];

    assign branch = (opcode == 7'b1100011);
    assign alu_src = (opcode == 7'b0000011 || opcode == 7'b0100011);
    assign mem_to_reg = (opcode == 7'b0000011);
    assign alu_op[0] = (opcode == 7'b1100011);
    assign alu_op[1] = (opcode == 7'b0110011);
    assign mem_read = (opcode == 7'b0000011);
    assign mem_write = (opcode == 7'b0100011);
    assign reg_write = (opcode == 7'b0110011 || opcode == 7'b0000011);

    // --------------------------------------------------------------------

    // --------------------------------------------------------------------
    /* ALU control unit:
     * ALU control unit generate alu_control signal which selects ALU operations
     * Generating control signals using alu_op, funct7, and funct3 fields
     */
    logic   [6:0]   funct7;
    logic   [2:0]   funct3;

    // COMPLETE THE ALU CONTROL UNIT HERE

    assign funct7 = inst[31:25];
    assign funct3 = inst[14:12];

    always_comb begin
        case (alu_op)
            2'b00: alu_control <= 4'b0010; // lw/sw (add)
            2'b01: alu_control <= 4'b0110; // beq (sub)
            default: case(funct3)
		3'b111: alu_control <= 4'b0000;
		3'b110: alu_control <= 4'b0001;
		default: case(funct7)
			7'b0000000: alu_control <= 4'b0010;
			7'b0100000: alu_control <= 4'b0110;
			default: alu_control <= 4'b0010; //?? base add
			endcase
		endcase
	endcase
    end

    // ---------------------------------------------------------------------


    // ---------------------------------------------------------------------
    /* Immediate generator:
     * Generating immediate value from inst[31:0]
     */
    logic   [63:0]  imm64;
    logic   [63:0]  imm64_branch;  // imm64 left shifted by 1
    logic   [11:0]  imm12;  // 12-bit immediate value extracted from inst

    // COMPLETE IMMEDIATE GENERATOR HERE

    assign imm12 = inst[31:20];

    always_comb begin
	case(opcode)
		7'b0000011: imm64 = {inst[31]? {20{1'b1}}:20'b0 , imm12};
		7'b0100011: imm64 = {inst[31]? 20'b1:20'b0 , inst[31:25], inst[11:7]};
		7'b1100011: imm64_branch = {inst[31]? 20'b1:20'b0 , inst[7], inst[30:25],inst[11:8],1'b0};
		default : imm64 = {32'b0};
	endcase
    end

    // ----------------------------------------------------------------------

    // Program counter
    logic   [63:0]  pc_curr, pc_next;
    logic   [63:0]  pc_next_plus4, pc_next_branch;

    assign pc_next_plus4 = pc_curr + 64'h4;    // FILL THIS

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            pc_curr <= 'b0;
        end else begin
            pc_curr <= pc_next;        // FILL THIS
        end
    end


    // MUXes:
    // COMPLETE MUXES HERE
    // PC_NEXT
    assign pc_next_branch = pc_curr + imm64_branch;   // FILL THIS

    logic sel_branch;
    assign sel_branch = (branch&&alu_zero);

    assign pc_next = (sel_branch == 1)? pc_next_branch : pc_next_plus4;
    // ALU inputs

	assign alu_in1 = rs1_dout;
	assign alu_in2 = (alu_src == 1)? imm64 : rs2_dout;

    // RF din
	assign rd_din = (mem_read == 1 && mem_to_reg == 1)? dmem_dout : alu_result;

    // COMPLETE CONNECTIONS HERE
    // imem
	assign imem_addr = pc_curr;


    // regfile
	assign rs1 = inst[19:15];
	assign rs2 = inst[24:20];
	assign rd = inst[11:7];
	
    // dmem

	assign dmem_addr = alu_result;
	assign dmem_din = (mem_write == 1)? rs2_dout : 'b0;

    // -----------------------------------------------------------------------
    /* Instantiation of datapath elements
     * All input/output ports should be connected
     */
    
    // IMEM
    imem #(
        .IMEM_DEPTH         (IMEM_DEPTH),
        .IMEM_ADDR_WIDTH    (IMEM_ADDR_WIDTH)
    ) u_imem_0 (
        .addr               ( imem_addr     ),
        .dout               ( inst          )
    );

    // REGFILE
    regfile #(
	.REG_WIDTH (REG_WIDTH) 
    ) u_regfile_0 (
	.clk (clk),
	.rs1 (rs1),
	.rs2 (rs2),
	.rd (rd),
	.rd_din (rd_din),
	.reg_write (reg_write),
	.rs1_dout (rs1_dout),
	.rs2_dout (rs2_dout)
);

    // ALU
    alu #(
	.REG_WIDTH (REG_WIDTH)
    ) u_alu_0 (
	.in1 (alu_in1),
	.in2 (alu_in2),
	.alu_control (alu_control),
	.result (alu_result),
	.zero (alu_zero)
);

    // DMEM
    dmem #(
	.DMEM_DEPTH (DMEM_DEPTH),
	.DMEM_ADDR_WIDTH (DMEM_ADDR_WIDTH)
    ) u_dmem_0 (
	.clk (clk),
	.addr (dmem_addr),
	.din (dmem_din),
	.mem_read (mem_read),
	.mem_write (mem_write),
	.dout (dmem_dout)
);

endmodule
