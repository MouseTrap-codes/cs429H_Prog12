// instruction decoder
module instruction_decoder(
    input  [31:0] in,       // 32-bit instruction
    output [4:0]  opcode,   // Bits [31:27]
    output [4:0]  rd,       // Bits [26:22]
    output [4:0]  rs,       // Bits [21:17]
    output [4:0]  rt,       // Bits [16:12]
    output [11:0] L         // Bits [11:0]
);
    assign opcode = in[31:27];
    assign rd     = in[26:22];
    assign rs     = in[21:17];
    assign rt     = in[16:12];
    assign L      = in[11:0];
endmodule

// alu
module alu (
    input  [4:0]  opcode,
    input  [63:0] op1,       // First operand
    input  [63:0] op2,       // Second operand
    input  [11:0] L,         // 12-bit literal/immediate
    output reg [63:0] result // Result
);
    always @(*) begin
        case (opcode)
            // Integer arithmetic
            5'h18: result = op1 + op2;                   // add
            5'h19: result = op1 + {{52{L[11]}}, L};             // addi
            5'h1a: result = op1 - op2;                   // sub
            5'h1b: result = op1 - {52'b0, L};             // subi
            5'h1c: result = op1 * op2;                   // mul
            5'h1d: result = op1 / op2;                   // div
            // Logical operations
            5'h0:  result = op1 & op2;                   // and
            5'h1:  result = op1 | op2;                   // or
            5'h2:  result = op1 ^ op2;                   // xor
            5'h3:  result = ~op1;                        // not (rt ignored)
            // Shift operations
            5'h4:  result = op1 >> op2;                  // shftr
            5'h5:  result = op1 >> L;                    // shftri
            5'h6:  result = op1 << op2;                  // shftl
            5'h7:  result = op1 << L;                    // shftli
            // Data movement
            5'h11: result = op1;                        // mov rd, rs
            5'h12: begin                                 // mov rd, L: update lower 12 bits
                      result = op1;
                      result[11:0] = L;
                   end
            default: result = 64'b0;
        endcase
    end
endmodule


// FPU
module fpu (
    input  [4:0]  opcode,
    input  [63:0] rs,         // Operand 1
    input  [63:0] rt,         // Operand 2
    input  [11:0] L,          // Literal
    output reg [63:0] result  // FPU result
);
    real op1, op2, res_real;
    always @(*) begin
        op1 = $bitstoreal(rs);
        op2 = $bitstoreal(rt);
        case (opcode)
            5'h14: res_real = op1 + op2; // addf
            5'h15: res_real = op1 - op2; // subf
            5'h16: res_real = op1 * op2; // mulf
            5'h17: res_real = op1 / op2; // divf
            default: res_real = 0.0;
        endcase
        result = $realtobits(res_real);
    end
endmodule


// regFile
module regFile (
    input         clk,
    input         reset,
    input  [63:0] data_in,   // Data to write
    input         we,        // Write enable
    input  [4:0]  rd,        // Write address
    input  [4:0]  rs,        // Read address 1
    input  [4:0]  rt,        // Read address 2
    output reg [63:0] rdOut, // Data out port for write-back
    output reg [63:0] rsOut, // Data out port A
    output reg [63:0] rtOut  // Data out port B
);
    reg [63:0] registers [0:31];
    integer i;
    
    // Write process (synchronous)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1)
                registers[i] <= 64'b0;
            registers[31] <= 64'h80000;
        end else if (we) begin
            registers[rd] <= data_in;
        end
    end
    
    // Combinational read
    always @(*) begin
        rdOut = registers[rd];
        rsOut = registers[rs];
        rtOut = registers[rt];
    end
endmodule


// memory
module memory(
   input clk,
   input reset,
   // Fetch interface:
   input  [31:0] fetch_addr,
   output [31:0] fetch_instruction,
   // Data load interface:
   input  [31:0] data_load_addr,
   output [63:0] data_load,
   // Store interface:
   input         store_we,
   input  [31:0] store_addr,
   input  [63:0] store_data
);
    parameter MEM_SIZE = 512*1024;  // 512 KB
    reg [7:0] bytes [0:MEM_SIZE-1];
    integer i;
    
    always @(posedge clk) begin
        if (reset) begin
            // Optionally, initialize memory here.
        end
        if (store_we) begin
            bytes[store_addr]     <= store_data[63:56];
            bytes[store_addr + 1] <= store_data[55:48];
            bytes[store_addr + 2] <= store_data[47:40];
            bytes[store_addr + 3] <= store_data[39:32];
            bytes[store_addr + 4] <= store_data[31:24];
            bytes[store_addr + 5] <= store_data[23:16];
            bytes[store_addr + 6] <= store_data[15:8];
            bytes[store_addr + 7] <= store_data[7:0];
        end
    end
    
    // Instruction fetch (big-endian)
    assign fetch_instruction = { 
        bytes[fetch_addr],
        bytes[fetch_addr+1],
        bytes[fetch_addr+2],
        bytes[fetch_addr+3]
    };
    
    // Data load (big-endian)
    assign data_load = { 
        bytes[data_load_addr],
        bytes[data_load_addr+1],
        bytes[data_load_addr+2],
        bytes[data_load_addr+3],
        bytes[data_load_addr+4],
        bytes[data_load_addr+5],
        bytes[data_load_addr+6],
        bytes[data_load_addr+7]
    };
endmodule


// fetch
module fetch(
    input  [31:0] PC,
    input  [31:0] fetch_instruction,
    output [31:0] instruction
);
    assign instruction = fetch_instruction;
endmodule


// control.sv
module control(
    input      [2:0] current_state,  // Global FSM state
    input            clk,
    input            reset,
    input  [31:0]    instruction,
    input  [31:0]    PC,
    input  [63:0]    opA,         // Data from register file (rs)
    input  [63:0]    opB,         // Data from register file (rt)
    input  [63:0]    data_load,   // Data loaded from memory 
    output reg [31:0] next_PC,
    output reg [63:0] exec_result,
    output reg        write_en,
    output reg [4:0]  write_reg,
    output reg [4:0]  rf_addrA,
    output reg [4:0]  rf_addrB,
    output reg        mem_we,
    output reg [31:0] mem_addr,
    output reg [63:0] mem_write_data,
    output reg [31:0] data_load_addr
);
    // decode
    wire [4:0] opcode, rd, rs, rt;
    wire [11:0] L;
    instruction_decoder dec (
       .in(instruction),
       .opcode(opcode),
       .rd(rd), .rs(rs), .rt(rt), .L(L)
    );

    // ALU / FPU outputs
    wire [63:0] alu_out, fpu_out;
    alu  alu_inst (.opcode(opcode), .op1(opA), .op2(opB), .L(L), .result(alu_out));
    fpu  fpu_inst (.opcode(opcode), .rs(opA), .rt(opB), .L(L), .result(fpu_out));

    // register‐file read addresses (unchanged)
    always @(*) begin
        rf_addrA = rs; rf_addrB = rt;
        case(opcode)
            5'h19,5'h1b,5'h5,5'h7,5'h12: rf_addrA = rd;
            5'hb:                      rf_addrB = rd;
            5'h8,5'h9:                 rf_addrA = rd;
            5'hc: begin rf_addrA = rd; rf_addrB = 5'd31; end
            5'hd:                      rf_addrA = 5'd31;
            5'he:                      rf_addrB = rd;
            5'h13: begin rf_addrA = rd; rf_addrB = rs; end
            default: ;
        endcase
    end

    // main control
    always @(*) begin
        // defaults
        next_PC        = PC + 4;
        exec_result    = 64'b0;
        write_en       = 1'b0;
        write_reg      = rd;
        mem_we         = 1'b0;
        mem_addr       = 32'b0;
        mem_write_data = 64'b0;
        data_load_addr = 32'b0;

        case (current_state)
        //--------------------------------------------------
        3'd2: begin // EXECUTE
            // only compute ALU/FPU, but do NOT write back here
            case (opcode)
                // ALU ops
                5'h18,5'h1a,5'h1c,5'h1d,
                5'h0,5'h1,5'h2,5'h3,5'h4,5'h6,
                5'h19,5'h1b,5'h5,5'h7,5'h12:
                    exec_result = alu_out;
                // FPU ops
                5'h14,5'h15,5'h16,5'h17:
                    exec_result = fpu_out;
                default: ;
            endcase
        end

        //--------------------------------------------------
        3'd3: begin // MEMORY
            case (opcode)
                5'h8:  next_PC = opA;
                5'h9:  next_PC = PC + opA[31:0];
                5'ha:  next_PC = PC + {{20{L[11]}},L};
                5'hb:  next_PC = (opA!=0) ? opB : PC+4;
                5'hc:  begin mem_we=1; mem_addr=opB-8; mem_write_data=PC+4; next_PC=opA; end
                5'hd:  begin data_load_addr=opA-8; next_PC=data_load[31:0]; end
                5'he:  next_PC = ($signed(opA)>$signed(opB))? rd : PC+4;
                5'h10: begin data_load_addr=opA+{{52{L[11]}},L}; exec_result=data_load; end
                5'h11: begin exec_result=opA; end
                5'h13: begin mem_we=1; mem_addr=opA+{{52{L[11]}},L}; mem_write_data=opB; end
                default: ;
            endcase
        end

        //--------------------------------------------------
        3'd4: begin // WRITEBACK
            // now actually write the value computed in EX or MEM
            case (opcode)
                // integer/immediate/logical shifts & mov/reg‐to‐reg
                5'h18,5'h19,5'h1a,5'h1b,5'h1c,5'h1d,
                5'h0,5'h1,5'h2,5'h3,5'h4,5'h5,5'h6,5'h7,5'h12: begin
                    write_en    = 1'b1;
                    write_reg   = rd;
                    // exec_result was set back in EX
                end
                // floating point
                5'h14,5'h15,5'h16,5'h17: begin
                    write_en    = 1'b1;
                    write_reg   = rd;
                end
                // load
                5'h10: begin
                    write_en    = 1'b1;
                    write_reg   = rd;
                    // exec_result was set in MEM
                end
                default: ;
            endcase
        end

        default: ;
        endcase
    end
endmodule

// tinker core
module tinker_core(
    input clk,
    input reset,
    output logic hlt
);
    // State encoding:
    typedef enum logic [2:0] {
        FETCH     = 3'd0,
        DECODE    = 3'd1,
        EXECUTE   = 3'd2,
        MEMORY    = 3'd3,
        WRITEBACK = 3'd4
    } state_t;

    state_t current_state, next_state;
    
    // Program Counter (PC) register.
    reg [31:0] PC;
    
    // FSM: State register update
    always @(posedge clk or posedge reset) begin
        if (reset)
            current_state <= FETCH;
        else
            current_state <= next_state;
    end

    // FSM: Next state logic (simple 5‑stage cycle; adjust if needed)
    always @(*) begin
        case (current_state)
            FETCH:     next_state = DECODE;
            DECODE:    next_state = EXECUTE;
            EXECUTE:   next_state = MEMORY;
            MEMORY:    next_state = WRITEBACK;
            WRITEBACK: next_state = FETCH;
            default:   next_state = FETCH;
        endcase
    end

    // Instantiate memory module.
    wire [31:0] fetch_instruction;
    wire [63:0] data_load;
    // Data load and store signals:
    wire [31:0] mem_data_load_addr;
    wire        mem_we;
    wire [31:0] mem_store_addr;
    wire [63:0] mem_store_data;
    
    memory memory (
        .clk(clk),
        .reset(reset),
        .fetch_addr(PC),
        .fetch_instruction(fetch_instruction),
        .data_load_addr(mem_data_load_addr),
        .data_load(data_load),
        .store_we(mem_we),
        .store_addr(mem_store_addr),
        .store_data(mem_store_data)
    );
    
    // Instantiate fetch module.
    wire [31:0] instruction;
    fetch fetch_inst (
        .PC(PC),
        .fetch_instruction(fetch_instruction),
        .instruction(instruction)
    );

    // halt bs
    always @(*) begin
        // only asser halt in DECODE or later
        if ((current_state == DECODE || current_state == EXECUTE)
        && instruction[31:27] == 5'h0f)
            hlt = 1;
        else 
            hlt = 0; 
    end 


    // Instantiate register file.
    wire [4:0]  rf_addrA;
    wire [4:0]  rf_addrB;
    wire [63:0] opA, opB;
    reg  [63:0] write_data;
    reg         write_en;
    reg  [4:0]  write_reg;
    
    regFile reg_file (
        .clk(clk),
        .reset(reset),
        .data_in(write_data),
        .we(write_en),
        .rd(write_reg),
        .rs(rf_addrA),
        .rt(rf_addrB),
        .rsOut(opA),
        .rtOut(opB),
        .rdOut()  // if you wish, you can capture the write-back output here
    );
    
    // Instantiate control module.
    wire [31:0] ctrl_next_PC;
    wire [63:0] ctrl_exec_result;
    wire        ctrl_write_en;
    wire [4:0]  ctrl_write_reg;
    wire [4:0]  ctrl_rf_addrA;
    wire [4:0]  ctrl_rf_addrB;
    wire        ctrl_mem_we;
    wire [31:0] ctrl_mem_addr;
    wire [63:0] ctrl_mem_write_data;
    wire [31:0] ctrl_data_load_addr;
    
    control ctrl_inst (
        .current_state(current_state),
        .clk(clk),
        .reset(reset),
        .instruction(instruction),
        .PC(PC),
        .opA(opA),
        .opB(opB),
        .data_load(data_load),
        .next_PC(ctrl_next_PC),
        .exec_result(ctrl_exec_result),
        .write_en(ctrl_write_en),
        .write_reg(ctrl_write_reg),
        .rf_addrA(ctrl_rf_addrA),
        .rf_addrB(ctrl_rf_addrB),
        .mem_we(ctrl_mem_we),
        .mem_addr(ctrl_mem_addr),
        .mem_write_data(ctrl_mem_write_data),
        .data_load_addr(ctrl_data_load_addr)
    );
    
    // Connect control outputs to register file and memory.
    assign rf_addrA = ctrl_rf_addrA;
    assign rf_addrB = ctrl_rf_addrB;
    
    always @(*) begin
        write_data = ctrl_exec_result;
        write_en   = ctrl_write_en;
        write_reg  = ctrl_write_reg;
    end
    
    assign mem_we             = ctrl_mem_we;
    assign mem_store_addr     = ctrl_mem_addr;
    assign mem_store_data     = ctrl_mem_write_data;
    assign mem_data_load_addr = ctrl_data_load_addr;
    
    // PC update.
    always @(posedge clk) begin
        if (reset)
            PC <= 32'h2000;
        else
            PC <= ctrl_next_PC;
    end
endmodule
