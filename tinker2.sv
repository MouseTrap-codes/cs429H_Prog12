// instruction_decoder.v
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

// alu.v
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
            5'h19: result = op1 + {{52{L[11]}}, L};       // addi
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

// fpu.v
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

// regFile.v
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
    
    // Initial register values: r0-r30 = 0 and r31 = MEMSIZE (here 0x80000)
    initial begin
        for (i = 0; i < 31; i = i + 1)
            registers[i] = 64'b0;
        registers[31] = 64'h80000;
    end
    
    // Synchronous write.
    always @(posedge clk) begin
        if (we) begin
            registers[rd] <= data_in;
        end
    end
    
    // Combinational read.
    always @(*) begin
        rdOut = registers[rd];
        rsOut = registers[rs];
        rtOut = registers[rt];
    end
endmodule

// memory.v
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
    (* keep *) reg [7:0] bytes [0:MEM_SIZE-1];
    integer i;
    
    always @(posedge clk) begin
        if (reset) begin
            // Optionally initialize memory.
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

// fetch.v
module fetch(
    input  [31:0] PC,
    input  [31:0] fetch_instruction,
    output [31:0] instruction
);
    assign instruction = fetch_instruction;
endmodule

module control(
    input clk,
    input reset,
    input [4:0] opcode,
    input [63:0] signExtendedLiteral,
    output reg writeToMemory,
    output reg readFromMemory,
    output reg writeToRegister,
    output reg shouldUpdatePC,
    output reg fetchStage,
    output reg hlt
);
    // Define FSM states.
    parameter FETCH = 3'd0, DECODE = 3'd1, ALU = 3'd2,
              LOADSTORE = 3'd3, REGFILE = 3'd4;
    reg [2:0] state, nextState;
    
    // State Transition Logic.
    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= FETCH;
        else
            state <= nextState;
    end
    
    always @(*) begin
        // Default outputs.
        fetchStage = (state == FETCH);
        writeToMemory = 1'b0;
        readFromMemory  = 1'b0;
        writeToRegister = 1'b0;
        shouldUpdatePC = 1'b0;
        hlt = 1'b0;
        
        case (state)
            FETCH: begin
                nextState = DECODE;
            end
            DECODE: begin
                case (opcode)
                    5'h0d, 5'h10, 5'h13: nextState = LOADSTORE;
                    5'h0f: begin
                        if (signExtendedLiteral[3:0] == 4'h0) begin
                            hlt = 1;
                            nextState = FETCH;
                            shouldUpdatePC = 0;
                        end else begin
                            nextState = FETCH;
                            shouldUpdatePC = 1;
                        end
                    end
                    default: nextState = ALU;
                endcase
            end
            ALU: begin
                // For branch/jump opcodes, update PC immediately.
                case (opcode)
                    5'h08, 5'h09, 5'h0a, 5'h0b, 5'h0e: begin
                        nextState = REGFILE;
                        shouldUpdatePC = 1;
                    end
                    5'h0c: nextState = LOADSTORE;
                    default: nextState = REGFILE;
                endcase
            end
            LOADSTORE: begin
                case (opcode)
                    5'h0c: begin
                        writeToMemory = 1;
                        nextState = REGFILE;
                        shouldUpdatePC = 1;
                    end
                    5'h0d: begin
                        readFromMemory = 1;
                        nextState = REGFILE;
                        shouldUpdatePC = 1;
                    end
                    5'h10: begin
                        readFromMemory = 1;
                        nextState = REGFILE;
                        shouldUpdatePC = 1;
                    end
                    5'h13: begin
                        writeToMemory = 1;
                        nextState = REGFILE;
                        shouldUpdatePC = 1;
                    end
                    default: begin
                        nextState = REGFILE;
                        shouldUpdatePC = 1;
                    end
                endcase
            end
            REGFILE: begin
                // Here, the register file write occurs.
                // Enable write for arithmetic and move instructions.
                case (opcode)
                    5'h00, 5'h01, 5'h02, 5'h03,
                    5'h04, 5'h05, 5'h06, 5'h07,
                    5'h10, 5'h11, 5'h12,
                    5'h14, 5'h15, 5'h16, 5'h17,
                    5'h18, 5'h19, 5'h1a, 5'h1b, 5'h1c, 5'h1d:
                        writeToRegister = 1;
                    default: writeToRegister = 0;
                endcase
                nextState = FETCH;
                shouldUpdatePC = 1;
            end
            default: nextState = FETCH;
        endcase
    end
endmodule


module tinker_core(input clk, input reset, output hlt);
    // State definitions.
    typedef enum logic [2:0] {
        FETCH,    // Read instruction from memory.
        DECODE,   // Latch instruction & decode.
        EXECUTE,  // Perform ALU/FPU operations.
        LOADSTORE,// Compute effective address and perform memory access.
        REGFILE   // Write result to register file and update PC.
    } state_t;
    
    state_t state, nextState;
    
    // Datapath registers.
    reg [63:0] programCounter;
    reg [31:0] instructionRegister;
    reg [63:0] resultReg;   // Latches ALU or memory result.
    
    // Halt flag.
    reg halted;
    
    // ---------------------------
    // Instantiate Memory.
    // ---------------------------
    wire [31:0] memInstruction;
    wire [63:0] memData;
    wire memWriteEnable;
    wire [63:0] memWriteData;
    wire [63:0] effectiveAddress;
    
    // Compute effective address:
    // When in FETCH state, use programCounter; otherwise, for load/store,
    // use computed address based on opcode and registers.
    wire fetchStage = (state == FETCH);
    // We'll later compute effectiveAddress based on opcode; for now:
    assign effectiveAddress = fetchStage ? programCounter :
                              (opcode == 5'h0c || opcode == 5'h0d) ? (stackPointer - 64'd8) :
                              (opcode == 5'h10) ? (dataOutputOne + signExtendedLiteral) :
                              (opcode == 5'h13) ? (dataOutputThree + signExtendedLiteral) :
                              programCounter;
                              
    memory memory(
        .clk(clk),
        .reset(reset),
        .fetch_addr(effectiveAddress),
        .fetch_instruction(memInstruction),
        .data_load_addr(effectiveAddress),
        .data_load(memData),
        .store_we(memWriteEnable),
        .store_addr(effectiveAddress),
        .store_data(memWriteData)
    );
    
    // ---------------------------
    // Instruction Register.
    // ---------------------------
    always @(posedge clk or posedge reset) begin
        if (reset)
            instructionRegister <= 32'b0;
        else if (state == FETCH)  // Latch the fetched instruction.
            instructionRegister <= memInstruction;
    end
    
    // ---------------------------
    // Decoder and Sign-Extension.
    // ---------------------------
    wire [4:0] opcode, rd, rs, rt;
    wire [11:0] l;
    decoder dec_inst(
        .instruction(instructionRegister),
        .opcode(opcode),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .l(l)
    );
    wire [63:0] signExtendedLiteral;
    assign signExtendedLiteral = {{52{l[11]}}, l};
    
    // ---------------------------
    // Register File.
    // ---------------------------
    wire [63:0] dataOutputOne, dataOutputTwo, dataOutputThree, stackPointer;
    // The register file writes when writeToRegister is asserted.
    // We will drive its write data from resultReg.
    registers regFile(
        .clk(clk),
        .reset(reset),
        .write(writeToRegister),
        .data_input(resultReg),
        .registerOne(rs),
        .registerTwo(rt),
        .registerThree(rd),
        .data_outputOne(dataOutputOne),
        .data_outputTwo(dataOutputTwo),
        .data_outputThree(dataOutputThree),
        .stackPointer(stackPointer)
    );
    
    // ---------------------------
    // ALU.
    // ---------------------------
    wire [63:0] aluOutput, updatedPC;
    alu alu_inst(
        .opcode(opcode),
        .inputDataOne(dataOutputOne),
        .inputDataTwo(dataOutputTwo),
        .inputDataThree(dataOutputThree),
        .signExtendedLiteral(signExtendedLiteral),
        .programCounter(programCounter),
        .result(aluOutput),
        .newProgramCounter(updatedPC)
    );
    
    // For loads, the write data to the regFile comes from memory.
    // Otherwise, it comes from the ALU.
    wire [63:0] writeDataRegister;
    assign writeDataRegister = (opcode == 5'h10) ? memData : aluOutput;
    
    // ---------------------------
    // Memory Write Data.
    // ---------------------------
    // For store opcodes.
    assign memWriteData = (opcode == 5'h0c) ? (programCounter + 64'd4) : dataOutputOne;
    // Memory write enable signal comes from the control unit.
    
    // ---------------------------
    // Control Unit.
    // ---------------------------
    // The control module here implements a simple FSM.
    wire writeToMemory, readFromMemory, writeToRegister;
    wire shouldUpdatePC, ctrlFetchStage, ctrlHlt;
    
    control ctrl_inst(
        .clk(clk),
        .reset(reset),
        .opcode(opcode),
        .signExtendedLiteral(signExtendedLiteral),
        .writeToMemory(writeToMemory),
        .readFromMemory(readFromMemory),
        .writeToRegister(writeToRegister),
        .shouldUpdatePC(shouldUpdatePC),
        .fetchStage(ctrlFetchStage),
        .hlt(ctrlHlt)
    );
    
    // ---------------------------
    // Halt Flag.
    // ---------------------------
    always @(posedge clk or posedge reset) begin
        if (reset)
            halted <= 1'b0;
        else if (ctrlHlt)
            halted <= 1'b1;
    end
    assign hlt = halted;
    
    // ---------------------------
    // FSM: Next-State Logic.
    // ---------------------------
    always @(*) begin
        case (state)
            FETCH:    nextState = DECODE;
            DECODE:   nextState = EXECUTE;
            EXECUTE:  nextState = ( (opcode == 5'h0c || opcode == 5'h0d || opcode == 5'h10 || opcode == 5'h13) ? LOADSTORE : REGFILE );
            LOADSTORE: nextState = REGFILE;
            REGFILE:  nextState = FETCH;
            default:  nextState = FETCH;
        endcase
    end
    
    // ---------------------------
    // FSM: Sequential (State) Updates & Datapath Latching.
    // ---------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= FETCH;
            programCounter <= 64'h2000;
            resultReg <= 64'b0;
        end else begin
            state <= nextState;
            case (state)
                FETCH: begin
                    // Instruction was latched in IR already.
                end
                DECODE: begin
                    // No pipeline register needed.
                end
                EXECUTE: begin
                    // Latch ALU result.
                    resultReg <= aluOutput;
                end
                LOADSTORE: begin
                    // For a load, the result comes from memory.
                    if (readFromMemory)
                        resultReg <= memData; 
                    // For a store, nothing is latched.
                end
                REGFILE: begin
                    // The register file is written (via control signal writeToRegister)
                    // PC update happens here.
                    if (shouldUpdatePC) begin
                        if (opcode == 5'h0d)
                            programCounter <= memData; // jump via loaded data.
                        else
                            programCounter <= updatedPC;
                    end
                end
            endcase
        end
    end
endmodule

