//=====================================================================
// FETCH MODULE
//=====================================================================
module fetch (
    input         clk,
    input         reset,
    input         branch,       // High if we should branch this cycle
    input  [63:0] branch_pc,    // Target address if branching
    output reg [63:0] pc        // Current Program Counter
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 64'h2000;     // On reset, PC = 0x2000
        end
        else if (branch) begin
            pc <= branch_pc;    // Branch override
        end
        else begin
            pc <= pc + 4;       // Default: increment by 4
        end
    end
endmodule

//=====================================================================
// CONTROL MODULE
//=====================================================================
// Decides branching, load/store, register writes, etc.
module control (
    input  [4:0]  opcode,   // from instruction decoder
    input  [63:0] rs_val,   // register rs contents
    input  [63:0] rt_val,   // register rt contents
    input  [63:0] rd_val,   // register rd contents (sometimes used in branch target)
    input  [63:0] pc,       // current PC
    output reg    branch,   // if 1, fetch uses branch_pc
    output reg [63:0] branch_pc, // next PC if branching
    output reg    mem_read, // if 1, read 64-bit data from memory
    output reg    mem_write,// if 1, write 64-bit data to memory
    output reg    reg_write // if 1, write result to register file
);
    always @(*) begin
        // Defaults
        branch     = 0;
        branch_pc  = 64'b0;
        mem_read   = 0;
        mem_write  = 0;
        reg_write  = 1;  // By default, instructions write to RD

        case (opcode)
            // Branch instructions (examples)
            5'b01000: begin // br rd => pc <- rd_val
                branch    = 1;
                branch_pc = rd_val;
            end
            5'b01001: begin // brr rd => pc <- pc + rd_val
                branch    = 1;
                branch_pc = pc + rd_val;
            end
            5'b01010: begin // brr L => pc <- pc + sign-extended L
                branch    = 1;
                // For simplicity, assume L is stored in rd_val[11:0]
                branch_pc = pc + {{52{1'b0}}, rd_val[11:0]};
            end
            5'b01011: begin // brnz => if (rs_val != 0) => pc <- rd_val
                if (rs_val != 0) begin
                    branch    = 1;
                    branch_pc = rd_val;
                end
            end
            5'b01110: begin // brgt => if (rs_val > rt_val) => pc <- rd_val
                if ($signed(rs_val) > $signed(rt_val)) begin
                    branch    = 1;
                    branch_pc = rd_val;
                end
            end

            // Loads / Stores (examples)
            // 0x10 => mov rd, (rs)(L) => load
            5'b10000: begin
                mem_read  = 1;  // load from memory
                reg_write = 1;  // write loaded data to RD
            end
            // 0x13 => mov (rd)(L), rs => store
            5'b10011: begin
                mem_write = 1;  // store data
                reg_write = 0;  // typically don't write to RD
            end

            default: begin
                // Normal ALU instruction => no branch
                // no special mem read/write
            end
        endcase
    end
endmodule

//=====================================================================
// MEMORY MODULE
//=====================================================================
// Single Von Neumann memory storing both instructions (4 bytes) and data (8 bytes).
module memory (
    input  [63:0] addr,
    input         mem_read_instr,  // 1 => fetch 32-bit instruction from addr
    input         mem_read_data,   // 1 => fetch 64-bit data from addr
    input         mem_write,       // 1 => write 64-bit data
    input  [63:0] write_data,      // data to store
    output [31:0] instr_out,       // 32-bit instruction
    output [63:0] data_out         // 64-bit data
);
    parameter MEM_SIZE = 524288;

    // The testbench expects this name:
    reg [7:0] bytes [0:MEM_SIZE-1];  // Must be named "bytes"

    // 4-byte instruction fetch
    assign instr_out = (mem_read_instr)
        ? { bytes[addr],
            bytes[addr+1],
            bytes[addr+2],
            bytes[addr+3] }
        : 32'b0;

    // 8-byte data fetch
    assign data_out = (mem_read_data)
        ? { bytes[addr],
            bytes[addr+1],
            bytes[addr+2],
            bytes[addr+3],
            bytes[addr+4],
            bytes[addr+5],
            bytes[addr+6],
            bytes[addr+7] }
        : 64'b0;

    // Write 8 bytes
    // For purely synchronous memory, you'd typically do @(posedge clk)
    always @(posedge mem_write) begin
        { bytes[addr],
          bytes[addr+1],
          bytes[addr+2],
          bytes[addr+3],
          bytes[addr+4],
          bytes[addr+5],
          bytes[addr+6],
          bytes[addr+7] } = write_data;
    end
endmodule

//=====================================================================
// REG_FILE MODULE
//=====================================================================
// Stores register contents in a packed array named `registers`.
module reg_file (
    input         clk,
    input  [63:0] data_in,    // Data to write
    input  [4:0]  write_reg,  // Destination register index
    input         we,         // Write enable
    input  [4:0]  rd,         // For reading rd
    input  [4:0]  rs,         // For reading rs
    input  [4:0]  rt,         // For reading rt
    output reg [63:0] rdOut,
    output reg [63:0] rsOut,
    output reg [63:0] rtOut
);
    // The testbench expects this name:
    reg [63:0] registers [0:31];  // Must be named "registers"

    // Synchronous write
    always @(posedge clk) begin
        if (we && (write_reg != 5'd0)) begin
            registers[write_reg] <= data_in;
        end
    end

    // Combinational reads
    always @(*) begin
        rdOut = registers[rd];
        rsOut = registers[rs];
        rtOut = registers[rt];
    end
endmodule

//=====================================================================
// INSTRUCTION DECODER MODULE
//=====================================================================
module instruction_decoder(
    input  [31:0] instr,
    output [4:0]  opcode,
    output [4:0]  rd,
    output [4:0]  rs,
    output [4:0]  rt,
    output [11:0] L
);
    assign opcode = instr[31:27];
    assign rd     = instr[26:22];
    assign rs     = instr[21:17];
    assign rt     = instr[16:12];
    assign L      = instr[11:0];
endmodule

//=====================================================================
// ALU MODULE
//=====================================================================
module alu(
    input  [4:0]  opcode,
    input  [63:0] rdVal,
    input  [63:0] rsVal,
    input  [63:0] rtVal,
    input  [11:0] L,
    output reg [63:0] aluResult
);
    always @(*) begin
        aluResult = 64'b0;
        case (opcode)
            5'b11000: aluResult = $signed(rsVal) + $signed(rtVal); // add
            5'b11001: aluResult = rsVal + L;     // addi
            5'b11010: aluResult = rsVal - rtVal; // sub
            5'b11011: aluResult = rsVal - L;     // subi
            5'b11100: aluResult = rsVal * rtVal; // mul
            5'b11101: aluResult = rsVal / rtVal; // div
            5'b00000: aluResult = rsVal & rtVal; // and
            5'b00001: aluResult = rsVal | rtVal; // or
            5'b00010: aluResult = rsVal ^ rtVal; // xor
            5'b00011: aluResult = ~rsVal;        // not
            5'b00100: aluResult = rsVal >> rtVal;// shftr
            5'b00101: aluResult = rsVal >> L;    // shftri
            5'b00110: aluResult = rsVal << rtVal;// shftl
            5'b00111: aluResult = rsVal << L;    // shftli
            5'b10001: aluResult = rsVal;         // mov rd, rs
            5'b10010: begin                      // mov rd, L
                aluResult = rdVal;
                aluResult[11:0] = L;
            end
        endcase
    end
endmodule

//=====================================================================
// FPU MODULE
//=====================================================================
module fpu(
    input  [4:0]  opcode,
    input  [63:0] rsVal,
    input  [63:0] rtVal,
    output reg [63:0] fpuResult
);
    real a, b, r;
    always @(*) begin
        a = $bitstoreal(rsVal);
        b = $bitstoreal(rtVal);
        r = 0.0;
        case (opcode)
            5'b10100: r = a + b;        // addf
            5'b10101: r = a - b;        // subf
            5'b10110: r = a * b;        // mulf
            5'b10111: if (b != 0.0) r = a / b; // divf
        endcase
        fpuResult = $realtobits(r);
    end
endmodule

//=====================================================================
// TOP-LEVEL MODULE (NAMED "core")
//=====================================================================
// Single-cycle design that instantiates fetch, control, memory, reg_file,
// instruction_decoder, ALU, FPU, etc. 
// On reset:
//  - PC <= 0x2000
//  - r31 <= 0x10000
module tinker_core (
    input clk,
    input reset
);
    //=================== FETCH ===================
    wire [63:0] pc;
    wire branch;
    wire [63:0] branch_pc;

    fetch fetch_inst (
        .clk(clk),
        .reset(reset),
        .branch(branch),
        .branch_pc(branch_pc),
        .pc(pc)
    );

    //=================== MEMORY ==================
    wire [31:0] instr_out;
    wire [63:0] mem_data_out;

    reg  [63:0] mem_addr;
    wire mem_read_instr = 1'b1;  // Always fetch instruction
    wire mem_read_data;
    wire mem_write;
    reg  [63:0] mem_write_data;

    // Instance name must be exactly "memory" for the testbench
    memory memory (
        .addr(mem_addr),
        .mem_read_instr(mem_read_instr),
        .mem_read_data(mem_read_data),
        .mem_write(mem_write),
        .write_data(mem_write_data),
        .instr_out(instr_out),
        .data_out(mem_data_out)
    );

    //=================== DECODER =================
    wire [4:0] opcode, rd, rs, rt;
    wire [11:0] L;
    instruction_decoder decoder_inst (
        .instr(instr_out),
        .opcode(opcode),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .L(L)
    );

    //=================== REGISTER FILE ===========
    wire [63:0] rdVal, rsVal, rtVal;
    reg  [63:0] write_data;
    reg  [4:0]  write_reg;
    reg         write_enable;

    // Instance name must be exactly "reg_file" for the testbench
    reg_file reg_file (
        .clk(clk),
        .data_in(write_data),
        .write_reg(write_reg),
        .we(write_enable),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .rdOut(rdVal),
        .rsOut(rsVal),
        .rtOut(rtVal)
    );

    //=================== ALU / FPU ===============
    wire [63:0] alu_result;
    alu alu_inst (
        .opcode(opcode),
        .rdVal(rdVal),
        .rsVal(rsVal),
        .rtVal(rtVal),
        .L(L),
        .aluResult(alu_result)
    );

    wire [63:0] fpu_result;
    fpu fpu_inst (
        .opcode(opcode),
        .rsVal(rsVal),
        .rtVal(rtVal),
        .fpuResult(fpu_result)
    );

    //=================== CONTROL =================
    wire mem_read, mem_write_en, reg_write_en;
    control control_inst (
        .opcode(opcode),
        .rs_val(rsVal),
        .rt_val(rtVal),
        .rd_val(rdVal),
        .pc(pc),
        .branch(branch),
        .branch_pc(branch_pc),
        .mem_read(mem_read),
        .mem_write(mem_write_en),
        .reg_write(reg_write_en)
    );

    //=================== MUX for Write-Back ======
    reg [63:0] mux_result;
    always @(*) begin
        // If floating opcode (101xx) => fpu_result
        if (opcode == 5'b10100 ||
            opcode == 5'b10101 ||
            opcode == 5'b10110 || 
            opcode == 5'b10111) begin
            mux_result = fpu_result;
        end
        // If mem_read => loaded data
        else if (mem_read) begin
            mux_result = mem_data_out;
        end
        // Otherwise => ALU result
        else begin
            mux_result = alu_result;
        end
    end

    //=================== Synchronous Block =======
    // On reset, set r31=0x10000
    // Decide memory address for load/store vs. instruction fetch
    // Drive register file writes, etc.
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // PC is already handled in fetch
            // Initialize r31 to 0x10000
            reg_file.registers[31] <= 64'h10000;
        end
        else begin
            // If we do load/store, address = ALU result
            // Else instruction fetch uses pc
            mem_addr <= pc; // default
            if (mem_read || mem_write_en) begin
                mem_addr <= alu_result;
            end

            // For store, we write from rsVal typically
            mem_write_data <= rsVal;

            // Register file write signals
            write_data   <= mux_result;
            write_reg    <= rd;
            write_enable <= reg_write_en && (rd != 5'd0);

            // Memory write
            // (wired above, just pass them)
        end
    end

    assign mem_write = mem_write_en;
endmodule
