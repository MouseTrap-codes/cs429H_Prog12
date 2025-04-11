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
// Decides branching, load/store, register writes, etc., and flags HALT.
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
    output reg    reg_write,// if 1, write result to register file
    output reg    hlt       // high when HALT instruction seen
);
    always @(*) begin
        // Defaults
        branch     = 0;
        branch_pc  = 0;
        mem_read   = 0;
        mem_write  = 0;
        reg_write  = 1;
        hlt        = 0;

        case (opcode)
            // === HALT ===
            5'b11111: begin
                // when we see HALT, assert hlt and do no other side‐effects
                hlt        = 1;
                reg_write  = 0;
            end

            // === Branch instructions ===
            5'b01000: begin // br rd => pc <- rd_val
                branch    = 1; branch_pc = rd_val;
            end
            5'b01001: begin // brr rd => pc <- pc + rd_val
                branch    = 1; branch_pc = pc + rd_val;
            end
            5'b01010: begin // brr L => pc <- pc + sign-extended L
                branch    = 1;
                branch_pc = pc + {{52{1'b0}}, rd_val[11:0]};
            end
            5'b01011: if (rs_val != 0) begin // brnz
                branch    = 1; branch_pc = rd_val;
            end
            5'b01110: if ($signed(rs_val) > $signed(rt_val)) begin // brgt
                branch    = 1; branch_pc = rd_val;
            end

            // === Loads / Stores ===
            5'b10000: begin // mov rd, (rs)(L)
                mem_read   = 1;
                reg_write  = 1;
            end
            5'b10011: begin // mov (rd)(L), rs
                mem_write  = 1;
                reg_write  = 0;
            end

            // === ALU / FPU / default ===
            default: begin
                // leave defaults
            end
        endcase
    end
endmodule

//=====================================================================
// REG_FILE MODULE (with reset)
//=====================================================================
module reg_file (
    input         clk,
    input         reset,      // synchronous reset for registers
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
    parameter MEM_SIZE = 524288;
    reg [63:0] registers [0:31];
    integer i;

    // On reset, clear r0–r30, set r31 = MEM_SIZE
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1)
                registers[i] <= 64'b0;
            registers[31] <= MEM_SIZE;
        end
        else if (we && (write_reg != 5'd0)) begin
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
// (Other modules: memory, instruction_decoder, alu, fpu)
//   — unchanged from your original submission —
//=====================================================================
// ... paste your memory, decoder, ALU, FPU here ...

//=====================================================================
// TOP-LEVEL MODULE (NAMED "tinker_core")
//=====================================================================
module tinker_core (
    input         clk,
    input         reset,
    output        hlt       // cycle-complete flag
);
    // FETCH
    wire [63:0] pc;
    wire        branch;
    wire [63:0] branch_pc;
    fetch fetch_inst (
        .clk(clk), .reset(reset),
        .branch(branch), .branch_pc(branch_pc),
        .pc(pc)
    );

    // MEMORY
    wire [31:0] instr_out;
    wire [63:0] mem_data_out;
    reg  [63:0] mem_addr;
    wire        mem_read_instr = 1'b1;  // always fetch
    wire        mem_read_data;
    wire        mem_write_en;
    reg  [63:0] mem_write_data;
    memory memory (
        .addr        (mem_addr),
        .mem_read_instr(mem_read_instr),
        .mem_read_data(mem_read_data),
        .mem_write   (mem_write_en),
        .write_data  (mem_write_data),
        .instr_out   (instr_out),
        .data_out    (mem_data_out)
    );

    // DECODER
    wire [4:0] opcode, rd, rs, rt;
    wire [11:0] L;
    instruction_decoder decoder_inst (
        .instr(instr_out),
        .opcode(opcode),
        .rd(rd), .rs(rs), .rt(rt),
        .L(L)
    );

    // REGISTER FILE
    wire [63:0] rdVal, rsVal, rtVal;
    reg  [63:0] write_data;
    reg  [4:0]  write_reg;
    reg         write_enable;
    reg_file reg_file_inst (
        .clk(clk), .reset(reset),
        .data_in(write_data),
        .write_reg(write_reg),
        .we(write_enable),
        .rd(rd), .rs(rs), .rt(rt),
        .rdOut(rdVal), .rsOut(rsVal), .rtOut(rtVal)
    );

    // ALU / FPU
    wire [63:0] alu_result, fpu_result;
    alu alu_inst (
        .opcode(opcode), .rsVal(rsVal), .rtVal(rtVal),
        .L(L), .aluResult(alu_result)
    );
    fpu fpu_inst (
        .opcode(opcode), .rsVal(rsVal), .rtVal(rtVal),
        .fpuResult(fpu_result)
    );

    // CONTROL
    wire        mem_read, mem_write, reg_write;
    wire        branch_ctrl;
    wire [63:0] branch_pc_ctrl;
    wire        hlt_ctrl;
    control control_inst (
        .opcode(opcode),
        .rs_val(rsVal), .rt_val(rtVal), .rd_val(rdVal),
        .pc(pc),
        .branch(branch_ctrl),
        .branch_pc(branch_pc_ctrl),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .reg_write(reg_write),
        .hlt(hlt_ctrl)
    );

    // Latch the hlt output
    reg hlt_r;
    always @(posedge clk or posedge reset) begin
        if (reset)       hlt_r <= 1'b0;
        else if (hlt_ctrl) hlt_r <= 1'b1;
    end
    assign hlt = hlt_r;

    // MUX for write‑back
    reg [63:0] mux_result;
    always @(*) begin
        if (opcode[4:2] == 3'b101)       mux_result = fpu_result;
        else if (mem_read)               mux_result = mem_data_out;
        else                             mux_result = alu_result;
    end

    // Synchronous updates: mem_addr, writeback, etc.
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_addr       <= 64'b0;
            mem_write_data <= 64'b0;
            write_data     <= 64'b0;
            write_reg      <= 5'b0;
            write_enable   <= 1'b0;
        end else begin
            // address: instruction fetch or data access
            mem_addr       <= (mem_read || mem_write) ? alu_result : pc;
            mem_write_data <= rsVal;
            write_data     <= mux_result;
            write_reg      <= rd;
            write_enable   <= reg_write && (rd != 5'd0);
        end
    end

    // Branch override
    assign branch    = branch_ctrl;
    assign branch_pc = branch_pc_ctrl;
    assign mem_write_en = mem_write;

endmodule
