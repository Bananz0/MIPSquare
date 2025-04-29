//
// Created by glenm on 4/16/2025.
//

#include "CpuSimulator.h"


CPUSimulator:: CPUSimulator() :
    regfile(new RegisterFile()),
    instructionMemory(new Memory()),
    dataMemory(new Memory()),
    pipelineStructure(new PipelineStructure()),
    alu(new ALU()),
    programCounter(new ProgramCounter()),
    mux1(new Multiplexer()),
    mux2(new Multiplexer()),
    mux3(new Multiplexer()),
    mux4(new Multiplexer()){
    if constexpr (DEBUG) {
        std::cout << std::endl << "Loading Instructions from glenOS 11" << std::endl;
    }
    //Load the program instructions into the memory
    loadProgramInstructions(loadProgramFromFile());

    if constexpr (DEBUG) {
        std::cout << "glenCoreUltra v1.0 (MIPS edition) initialized\n" << std::endl;
    }
}

CPUSimulator::~CPUSimulator() {
    //I have obviously not done anything. This is for my sanity
    if constexpr (DEBUG) {
        std::cout << "CPU Simulator destroyed, memory freed." << std::endl;
    }
}

std::vector<uint32_t> CPUSimulator::loadProgramFromFile() {
    std::vector<int32_t> MIPSProgram;
    MipsParser parser;
    // Load and parse the assembly file into machine code
    std::vector<uint32_t> machineCode = parser.loadProgramFromFile("MIPSProgram.txt");

    if (machineCode.empty()) {
        std::cerr << "Error: Failed to parse MIPS program or file is empty" << std::endl;
        return {};
    }

    // Print the parsed instructions for debugging
    if constexpr (PROG_VERBOSE) {
        std::cout << "===============================" << std::endl;
        std::cout << "Assembly to Machine Code Translation" << std::endl;
        std::cout << "===============================" << std::endl;

        for (size_t i = 0; i < machineCode.size(); i++) {
            std::cout << "Instruction " << i << ": 0x" << std::hex << machineCode[i] << std::dec << std::endl;

            // Create an instruction object to decode and display
            Instruction instr(machineCode[i]);
            std::cout << "  Decoded: " << instr.toString() << std::endl;
            std::cout << "  Binary: " << std::bitset<32>(machineCode[i]) << std::endl;
            std::cout << std::endl;
        }

        std::cout << "===============================" << std::endl;
    }

    return machineCode;
    // //Open the MIPS Program File
    // std::ifstream programRaw("MIPSProgram.txt");
    // if (!programRaw.is_open()) {
    //     std::cerr << "Error: Could not open MIPSProgram.txt\n" << std::endl;
    //     return {};
    // }
    // int myText;
    // while (programRaw >> std::hex >> myText) {
    //     MIPSProgram.push_back(myText);
    //     // if constexpr (PROG_VERBOSE) {
    //     //     printf("Read Value (HEX): 0x%x\n", myText);
    //     // }
    // }
    // if constexpr (PROG_VERBOSE) {
    //     printInstructions(MIPSProgram);
    // }
    // //Close the file
    // programRaw.close();
    // return MIPSProgram;
}

void CPUSimulator::loadProgramInstructions(const std::vector<uint32_t> memData) {
    if constexpr (DEBUG) {
        std::cout << std::endl << "Pushing Instrictions to instructionMemory" << std::endl;
    }
    instructionMemory->setMemory(memData);
    programLoaded = true;
}

void CPUSimulator::printInstructions(const std::vector<int32_t> &instructionVector) {
    //Used mainly for testng if the read function works and it does
    std::cout << "===============================" << std::endl;
    std::cout << "Instructions Read from File" << std::endl;
    std::cout << "===============================" << std::endl;
    for (size_t i = 0; i < instructionVector.size(); i++) {
        std::cout << "Instruction "<< i << ": 0x"<< std::hex <<  instructionVector[i] <<std::endl;
        Instruction pushedInstruction(instructionVector[i]);
        std::cout << pushedInstruction.toString() << std::endl;
    }
    std::cout << "===============================" << std::endl;
    std::cout << "===========    END  ===========" << std::endl;
    std::cout << "===============================" << std::endl;
}

void CPUSimulator::fetch() const {
    // Check if pipeline is stalled
    if (pipelineStructure->stallPipeline) {
        // Don't fetch new instruction or increment PC when stalled
        if constexpr (DEBUG) {
            std::cout << "Pipeline stalled due to load-use hazard in Fetch" << std::endl;
        }
        pipelineStructure->if_id.valid = false;
        pipelineStructure->IF_Done = false;
        return;
    }

    // Get current PC value
    int32_t currentPC = programCounter->getPC();

    // Check if we've reached the end of instruction memory
    if (currentPC >= instructionMemory->getMemory().size() * 4) {
        pipelineStructure->if_id.valid = false;
        pipelineStructure->IF_Done = true;
        return;
    }

    std::cout << "=========================== ====" << std::endl;
    std::cout << "=========== CPU Fetch  =========" << std::endl;

    // Fetch instruction from memory
    int32_t instructionFetch = instructionMemory->getMemoryValue(currentPC);


    // Write to if_id buffer
    pipelineStructure->next_if_id.pc = currentPC;
    pipelineStructure->next_if_id.instruction = Instruction(instructionFetch);
    pipelineStructure->next_if_id.valid = true;


    if constexpr (DEBUG) {
        std::cout << "CPU fetched instruction at PC: 0x" << std::hex << currentPC << std::dec
                << std::endl;
        std::cout << pipelineStructure->if_id.instruction.toString() << std::endl;
    }

    programCounter->incrementPC();
    pipelineStructure->IF_Done = pipelineStructure->if_id.valid;
    std::cout << "+++++++++++++++++++++++++++++++++++++++" << std::endl;
}

void CPUSimulator::handleBranchHazard(bool taken, int32_t target_pc) const {
    if (taken) {
        //Update PC to branch target
        programCounter->setPC(target_pc);

        //Flush pipeline stages
        pipelineStructure->next_if_id.valid = false; // Invalidate IF/ID
        pipelineStructure->next_id_ex.valid = false; // Also invalidate ID/EX for proper flush

        //Reset the pipeline stage flags for flushed instructions
        pipelineStructure->IF_Done = false;
        pipelineStructure->ID_Done = false;

        if constexpr (DEBUG) {
            std::cout << "Branch taken. Flushing pipeline and jumping to 0x" <<
                    std::hex << target_pc << std::dec << std::endl;
        }
    }
}


void CPUSimulator::decode() const {
    // If a stall is active, maintain the state and don't progress
    if (pipelineStructure->stallPipeline) {
        // Don't update ID/EX when stalled - preserve current IF/ID state
        if constexpr (DEBUG) {
            std::cout << "Pipeline stalled due to load-use hazard in Decode" << std::endl;
        }
        pipelineStructure->ID_Done = false;
        return;
    }

    // If there's no valid instruction in IF/ID
    if (!pipelineStructure->if_id.valid) {
        //Checks if the instruction in If/Id is a valid instruction
        pipelineStructure->next_id_ex.valid = false; //If not, pass a bubble (NOP) downstream
        pipelineStructure->ID_Done = true;
        return;
    }

    // Check for STOP instruction
    if (pipelineStructure->if_id.valid && pipelineStructure->if_id.instruction.getRawInstruction() == 0xFFFFFFFF) {
        std::cout << "\n==================================" << std::endl;
        std::cout << "STOP instruction detected. Terminating program." << std::endl;
        std::cout << "==================================" << std::endl;

        // Make this instruction invalid so it doesn't propagate
        pipelineStructure->if_id.valid = false;
        pipelineStructure->next_id_ex.valid = false;

        const_cast<CPUSimulator*>(this)->cpuRunning = false;

        pipelineStructure->ID_Done = true;
        return;
    }

    std::cout << "==========   =====================" << std::endl;
    std::cout << "===========    DECODE  ===========" << std::endl;

    // Extract instruction fields
    Instruction &instr = pipelineStructure->if_id.instruction;
    int32_t rs_value = regfile->getRegisterValue(instr.getRs());
    int32_t rt_value = regfile->getRegisterValue(instr.getRt());

    if constexpr (DEBUG) {
        std::cout << "\nCPU Decoding: " << instr.toString() << std::endl;
        std::cout << "Decode: reading from $" << getRegisterName(instr.getRs()) << std::endl;
        std::cout << "Decode: reading from $" << getRegisterName(instr.getRt()) << std::endl;
    }


    // Initialize all the values
    int8_t aluOp = 0;
    bool aluSrc = false;
    bool regWrite = false;
    bool memRead = false;
    bool memWrite = false;
    bool memToReg = false;
    bool branch = false;
    bool jump = false;
    int8_t regDst = 0;

    switch (instr.getOpcode()) {
        case 0x08: //ADDI
            aluOp = 0x0; // Add
            aluSrc = true; // Immediate value
            regWrite = true;
            regDst = 0; // Destination is rt (fixed)
            break;
        case 0x23: //LW
            memRead = true;
            regWrite = true;
            memToReg = true;
            aluOp = 0x0; //Add base and offset
            aluSrc = true; //alu source is immediate
            regDst = 0; //Destination is rt
            break;
        case 0x2B: //SW
            memWrite = true;
            aluOp = 0x0; //Add base and offset
            aluSrc = true; //Immediate Value
            break;
        case 0x4: //BEQ
            branch = true;
            aluOp = 0x1; //Substract for branch
            break;
        case 0x0: //R-type instructions
            switch (instr.getFunct()) {
                case 0x20: //ADD
                    aluOp = 0x0; //Add
                    regWrite = true;
                    regDst = 1; //Destination is rd
                    break;
                case 0x00: //SLL
                    aluOp = 0x4;
                    regWrite = true;
                    regDst = 1; // Destination is rd
                    break;
            }
            break;
        case 0x02: // J
            jump = true;
            break;
        case 0x03: // JAL
            jump = true;
            regWrite = true;
            regDst = 2; // $ra
            break;
        case 0x05: // BNE
            branch = true;
            aluOp = 0xC;
            break;
        case 0x25: //SLTIU
            aluOp = 0x2; //Set less than Immediate unsignd
            aluSrc = true; //Immiedate value
            regWrite = true;
            regDst = 0; // Destination is rt
            break;
        default:
            std::cerr << "Unsupported opcode: 0x" << std::hex << instr.getOpcode() << std::dec << std::endl;
            regWrite = false;
            memWrite = false;
            branch = false;
            jump = false;
            break;
    }

    // Check for load-use hazard here before updating ID/EX
    if (detectLoadUseHazard()) {
        if constexpr (DEBUG) {
            std::cout << "Load-use hazard detected in decode stage" << std::endl;
        }
        // Don't update downstream pipeline registers - handled in detectLoadUseHazard
        pipelineStructure->ID_Done = false;
        return;
    }
    //Create a copy of these values, so we can change it later on
    int32_t corrected_rs_value = rs_value;
    int32_t corrected_rt_value = rt_value;


    // EX/MEM Forwarding
    if (pipelineStructure->ex_mem.valid && pipelineStructure->ex_mem.regWrite) {
        bool can_forward = !(pipelineStructure->ex_mem.memToReg && pipelineStructure->ex_mem.memRead);
        //No load instruction
        int32_t forward_data = pipelineStructure->ex_mem.alu_result;
        int32_t ex_mem_dest_reg = pipelineStructure->ex_mem.rd_num;

        if (can_forward) {
            // Forward to RS
            if (instr.getRs() == ex_mem_dest_reg && instr.getRs() != 0) {
                corrected_rs_value = forward_data;
            }

            // Forward to RT
            if (instr.getRt() == ex_mem_dest_reg && instr.getRt() != 0) {
                corrected_rt_value = forward_data;
            }
        }
    }

    // MEM/WB Forwarding
    if (pipelineStructure->mem_wb.valid && pipelineStructure->mem_wb.regWrite) {
        int32_t wb_data = pipelineStructure->mem_wb.memToReg
                               ? pipelineStructure->mem_wb.memory_read_data
                               : pipelineStructure->mem_wb.alu_result;

        int32_t mem_wb_dest_reg = pipelineStructure->mem_wb.rd_num;

        // Forward to RS if needed
        if (instr.getRs() == mem_wb_dest_reg && instr.getRs() != 0) {
            corrected_rs_value = wb_data;
        }

        // Forward to RT if needed
        if (instr.getRt() == mem_wb_dest_reg && instr.getRt() != 0) {
            corrected_rt_value = wb_data;
        }
    }

    if (jump) {
        std::cout << "Jump instruction detected in decode: "
                << instr.toString() << std::endl;
        std::cout << "Jump target: 0x" << std::hex
                << instr.getJumpTarget() << std::dec << std::endl;

        // Set PC directly for jumps
        int32_t jumpAddr = (programCounter->getPC() & 0xF0000000) | (instr.getJumpTarget() << 2);
        std::cout << "Jumping to address: 0x" << std::hex << jumpAddr << std::dec << std::endl;

        // Set the PC and flush the pipeline
        programCounter->setPC(jumpAddr);
        pipelineStructure->if_id.valid = false; // Invalidate fetched instruction

        // Indicate jump was taken
        pipelineStructure->jumpTaken = true;
        std::cout << "Jump taken. PC set to: 0x" << std::hex
                << programCounter->getPC() << std::hex << std::endl;
    }

    //Updating ID/EX register
    pipelineStructure->next_id_ex.pc = pipelineStructure->if_id.pc;
    pipelineStructure->next_id_ex.instruction = instr;
    pipelineStructure->next_id_ex.rs_num = instr.getRs();
    pipelineStructure->next_id_ex.rt_num = instr.getRt();
    pipelineStructure->next_id_ex.rd_num = instr.getRd();
    pipelineStructure->next_id_ex.rs_value = corrected_rs_value; // Use corrected value
    pipelineStructure->next_id_ex.rt_value = corrected_rt_value; // Use corrected value
    pipelineStructure->next_id_ex.immediate = instr.getImmediate();
    pipelineStructure->next_id_ex.shamt = instr.getShamt();
    //Control Signals
    pipelineStructure->next_id_ex.memToReg = memToReg;
    pipelineStructure->next_id_ex.branch = branch;
    pipelineStructure->next_id_ex.jump = jump;
    pipelineStructure->next_id_ex.aluOp = aluOp;
    pipelineStructure->next_id_ex.aluSrc = aluSrc;
    pipelineStructure->next_id_ex.regWrite = regWrite;
    pipelineStructure->next_id_ex.memRead = memRead;
    pipelineStructure->next_id_ex.memWrite = memWrite;
    pipelineStructure->next_id_ex.regDst = regDst;
    pipelineStructure->next_id_ex.valid = true;

    pipelineStructure->ID_Done = true;

    if constexpr (DEBUG) {
        std::cout << "Decode complete - instruction moving to ID/EX stage" << std::endl;
    }
    std::cout << "+++++++++++++++++++++++++++++++++++++" << std::endl;
}

void CPUSimulator::execute() {
    // Only process if there's a valid instruction in ID/EX
    if (!pipelineStructure->id_ex.valid) {
        pipelineStructure->next_ex_mem.valid = false;
        pipelineStructure->EX_Done = true;
        return;
    }
    std::cout << "============================ == =" << std::endl;
    std::cout << "===========CPU EXECUTE===========" << std::endl;

    Instruction &instr = pipelineStructure->id_ex.instruction;

    if (instr.getOpcode() == 0 && instr.getFunct() == InstructionSet::NOP) {
        // Just pass the NOP through the pipeline
        pipelineStructure->ex_mem = pipelineStructure->id_ex;
        pipelineStructure->ex_mem.valid = true;
        pipelineStructure->EX_Done = true;

        if constexpr (DEBUG) {
            std::cout << "\nExecuting: NOP instruction" << std::endl;
        }

        std::cout << "+++++++++++++++++++++++++++++++++++++" << std::endl;
        return;
    }


    if (pipelineStructure->stallPipeline && !pipelineStructure->id_ex.memRead) {
        if constexpr (DEBUG) {
            std::cout << "Warning: Non-load instruction in execute during stall" << std::endl;
        }
    }

    int32_t destReg = alu->getDestinationRegister(
        pipelineStructure->id_ex.regDst,
        pipelineStructure->id_ex.rt_num,
        pipelineStructure->id_ex.rd_num
    );


    // Debug ID/EX register values
    std::cout << "ID/EX Stage Register Values:" << std::endl;
    std::cout << "  RS NUM: " << pipelineStructure->id_ex.rs_num << " (register: " << getRegisterName(
        pipelineStructure->id_ex.rs_num) << ")" << std::endl;
    std::cout << "  RS VALUE: " << pipelineStructure->id_ex.rs_value << std::endl;
    std::cout << "  RT NUM: " << pipelineStructure->id_ex.rt_num << " (register: " << getRegisterName(
        pipelineStructure->id_ex.rt_num) << ")" << std::endl;
    std::cout << "  RT VALUE: " << pipelineStructure->id_ex.rt_value << std::endl;
    std::cout << "  RD NUM: " << pipelineStructure->id_ex.rd_num << " (register: " << getRegisterName(
        pipelineStructure->id_ex.rd_num) << ")" << std::endl;
    std::cout << "  ALU SRC: " << (pipelineStructure->id_ex.aluSrc ? "Immediate" : "Register") << std::endl;
    std::cout << "  IMMEDIATE: " << pipelineStructure->id_ex.immediate << std::endl;


    // Get ALU inputs with forwarding
    int32_t aluInput1 = pipelineStructure->id_ex.rs_value;
    int32_t aluInput2;

    // For shift instructions, use shamt field
    //I'm not actually using this but had it to test my capability and it not worky
    if (pipelineStructure->id_ex.instruction.getOpcode() == 0x0 &&
        (pipelineStructure->id_ex.instruction.getFunct() == 0x00 || // SLL
         pipelineStructure->id_ex.instruction.getFunct() == 0x02 || // SRL
         pipelineStructure->id_ex.instruction.getFunct() == 0x03)) // SRA
    {
        // For shift instructions, rs is actually rt, and shamt is the shift amount
        aluInput1 = pipelineStructure->id_ex.rt_value;
        aluInput2 = pipelineStructure->id_ex.shamt;
    } else {
        // Normal ALU operation
        if (pipelineStructure->id_ex.aluSrc) {
            // Sign extend the immediate value before passing it to ALU
            uint16_t bit = pipelineStructure->id_ex.immediate;
            int32_t sigX = (bit & 0x8000) ? (0xFFFF0000 | bit) : bit;
            aluInput2 = sigX;
        } else {
            aluInput2 = pipelineStructure->id_ex.rt_value;
        }
    }

    // Apply forwarding logic
   dataForwarder(aluInput1, aluInput2);

    // Execute ALU operation
    bool branchTaken = false;
    int32_t aluResult = alu->execute(
        pipelineStructure->id_ex.aluOp,
        aluInput1,
        aluInput2,
        pipelineStructure->id_ex.pc,
        branchTaken
    );

    if constexpr (DEBUG) {
        std::cout << "ALU Result: " << std::dec << aluResult << std::dec << std::endl;
    }

    // Update EX/MEM pipeline register
    pipelineStructure->next_ex_mem.pc = pipelineStructure->id_ex.pc;
    pipelineStructure->next_ex_mem.instruction = pipelineStructure->id_ex.instruction;
    pipelineStructure->next_ex_mem.rs_num = pipelineStructure->id_ex.rs_num;
    pipelineStructure->next_ex_mem.rt_num = pipelineStructure->id_ex.rt_num;
    pipelineStructure->next_ex_mem.rd_num = destReg; // Use calculated destination register
    pipelineStructure->next_ex_mem.alu_result = aluResult;
    pipelineStructure->next_ex_mem.rs_value = aluInput1; // Store forwarded values
    pipelineStructure->next_ex_mem.rt_value = (pipelineStructure->id_ex.aluSrc)
                                                  ? pipelineStructure->id_ex.rt_value
                                                  : // Original rt value for store
                                                  aluInput2; // Forwarded rt value
    pipelineStructure->next_ex_mem.immediate = pipelineStructure->id_ex.immediate;

    // Pass control signals
    pipelineStructure->next_ex_mem.regWrite = pipelineStructure->id_ex.regWrite;
    pipelineStructure->next_ex_mem.memRead = pipelineStructure->id_ex.memRead;
    pipelineStructure->next_ex_mem.memWrite = pipelineStructure->id_ex.memWrite;
    pipelineStructure->next_ex_mem.memToReg = pipelineStructure->id_ex.memToReg;
    pipelineStructure->next_ex_mem.jump = pipelineStructure->id_ex.jump;
    pipelineStructure->next_ex_mem.valid = true;


    // Handle branch instructions
    if (pipelineStructure->id_ex.branch) {
        // Get ALU inputs with forwarding (make sure these are up to date from dataForwarder)
        int32_t rs_value = aluInput1;
        int32_t rt_value = aluInput2;

        branchTaken = false;

        // For BEQ, branch if rs == rt
        if (pipelineStructure->id_ex.instruction.getOpcode() == 0x4) {
            // BEQ
            branchTaken = (rs_value == rt_value);
        }
        // For BNE, branch if rs != rt
        else if (pipelineStructure->id_ex.instruction.getOpcode() == 0x5) {
            // BNE
            branchTaken = (rs_value != rt_value);
        }

        if (branchTaken) {
            // Calculate branch target: PC + 4 + (sign-extended immediate << 2)
            int32_t branchTarget = pipelineStructure->id_ex.pc + 4 +
                                    (pipelineStructure->id_ex.immediate << 2);

            // Debug info
            if constexpr (DEBUG) {
                std::cout << "Branch condition met! Taking branch to 0x" << std::hex << branchTarget << std::dec <<
                        std::endl;
                std::cout << "RS value: " << rs_value << ", RT value: " << rt_value << std::endl;
            }

            handleBranchHazard(true, branchTarget);
        } else {
            if constexpr (DEBUG) {
                std::cout << "Branch condition not met. Continuing sequential execution." << std::endl;
                std::cout << "RS value: " << rs_value << ", RT value: " << rt_value << std::endl;
            }
        }
    }

    // Handle jump instructions
    if (pipelineStructure->id_ex.jump && !pipelineStructure->jumpTaken) {
        int32_t jumpTarget;

        if (pipelineStructure->id_ex.aluOp == 0xF) {
            // JR
            jumpTarget = aluInput1; // Jump to address in register
            if constexpr (DEBUG) {
                std::cout << "Executing JR instruction." << std::endl;
                std::cout << "  Jump register (RS): " << getRegisterName(pipelineStructure->id_ex.rs_num)
                          << ", Value: 0x" << std::hex << aluInput1 << std::dec << std::endl;
                std::cout << "  Calculated Jump Target: 0x" << std::hex << jumpTarget << std::dec << std::endl;
            }
        } else if (pipelineStructure->id_ex.aluOp == 0x20 || // J
                   pipelineStructure->id_ex.aluOp == 0x03) {  // JAL

            if (pipelineStructure->id_ex.aluOp == 0x12){
                if constexpr (DEBUG) {
                    std::cout << "Executing J instruction." << std::endl;
                }
            }
            else if (pipelineStructure->id_ex.aluOp == 0x13){
                 if constexpr (DEBUG) {
                    std::cout << "Executing JAL instruction." << std::endl;
                }
            }
            // Jump target from J-type format: PC[31:28] | (immediate << 2)
            int32_t pcHigh4 = pipelineStructure->id_ex.pc & 0xF0000000;
            int32_t targetOffset = pipelineStructure->id_ex.instruction.getJumpTarget() << 2;
            jumpTarget = pcHigh4 | targetOffset;

            if constexpr (DEBUG) {
                std::cout << "  PC[31:28]: 0x" << std::hex << pcHigh4 << std::dec << std::endl;
                std::cout << "  Target Offset (Immediate << 2): 0x" << std::hex << targetOffset << std::dec << std::endl;
                std::cout << "  Calculated Jump Target: 0x" << std::hex << jumpTarget << std::dec << std::endl;
            }
        } else {
            jumpTarget = pipelineStructure->id_ex.pc + 4; // Default fallback
            if constexpr (DEBUG) {
                std::cout << "Warning: Unknown jump type, falling back to PC + 4" << std::endl;
            }
        }

        if constexpr (DEBUG) {
            std::cout << "Final jump target for opcode 0x" << std::hex << (int)pipelineStructure->id_ex.instruction.getOpcode()
                      << ": 0x" << std::hex << jumpTarget << std::dec << std::endl;
        }
        handleBranchHazard(true, jumpTarget); // Use branch hazard handler for jumps too
    }
    pipelineStructure->jumpTaken = false;
    pipelineStructure->EX_Done = true;


    std::cout << "+++++++++++++++++++++++++++++++++++++" << std::endl;
}

void CPUSimulator::memoryAccess() const {
    // Don't proceed if EX stage is not done
    if (!pipelineStructure->EX_Done) {
        pipelineStructure->MEM_Done = false;
        return;
    }


    // Check if the instruction in EX/MEM is valid
    if (!pipelineStructure->ex_mem.valid) {
        // Bubble in the pipeline
        pipelineStructure->next_mem_wb.valid = false;
        pipelineStructure->MEM_Done = true;
        return;
    }

    std::cout << "=-===================================" << std::endl;
    std::cout << "=========CPU Memory Access+++++++++++" << std::endl;

    int32_t memoryData = 0;
    int32_t address = pipelineStructure->ex_mem.alu_result;
    int8_t opcode = pipelineStructure->ex_mem.instruction.getOpcode();

    // Memory read operations
    if (pipelineStructure->ex_mem.memRead) {
        switch (opcode) {
            case 0x23: // LW
                memoryData = dataMemory->getMemoryValue(address);
                break;

            case 0x20: // LB (Load Byte)
            {
                int32_t wordData = dataMemory->getMemoryValue(address & ~0x3); // Align to word boundary
                int8_t byteOffset = address & 0x3; // Get byte offset within word
                int8_t byteData = (wordData >> (byteOffset * 8)) & 0xFF;
                // Sign extend
                memoryData = (byteData & 0x80) ? (0xFFFFFF00 | byteData) : byteData;
            }
            break;

            case 0x24: // LBU (Load Byte Unsigned)
            {
                int32_t wordData = dataMemory->getMemoryValue(address & ~0x3);
                int8_t byteOffset = address & 0x3;
                memoryData = (wordData >> (byteOffset * 8)) & 0xFF; // No sign extension
            }
            break;

            case 0x21: // LH (Load Halfword)
            {
                int32_t wordData = dataMemory->getMemoryValue(address & ~0x3);
                int8_t halfwordOffset = (address & 0x2) >> 1; // 0 for lower halfword, 1 for upper
                int16_t halfwordData = (wordData >> (halfwordOffset * 16)) & 0xFFFF;
                // Sign extend
                memoryData = (halfwordData & 0x8000) ? (0xFFFF0000 | halfwordData) : halfwordData;
            }
            break;

            case 0x25: // LHU (Load Halfword Unsigned)
            {
                int32_t wordData = dataMemory->getMemoryValue(address & ~0x3);
                int8_t halfwordOffset = (address & 0x2) >> 1;
                memoryData = (wordData >> (halfwordOffset * 16)) & 0xFFFF; // No sign extension
            }
            break;

            default:
                std::cerr << "Unknown memory read operation for opcode: 0x" <<
                        std::hex << static_cast<int>(opcode) << std::dec << std::endl;
                memoryData = dataMemory->getMemoryValue(address); // Default to word read
        }
    }

    // Memory write operations
    if (pipelineStructure->ex_mem.memWrite) {
        int32_t writeData = pipelineStructure->ex_mem.rt_value;

        switch (opcode) {
            case 0x2B: // SW
                dataMemory->setMemoryValue(address, writeData);
                break;

            case 0x28: // SB (Store Byte)
            {
                int32_t wordData = dataMemory->getMemoryValue(address & ~0x3);
                int8_t byteOffset = address & 0x3;
                int32_t byteMask = ~(0xFF << (byteOffset * 8));
                int32_t newData = (wordData & byteMask) |
                                   ((writeData & 0xFF) << (byteOffset * 8));
                dataMemory->setMemoryValue(address & ~0x3, newData);
            }
            break;

            case 0x29: // SH (Store Halfword)
            {
                int32_t wordData = dataMemory->getMemoryValue(address & ~0x3);
                int8_t halfwordOffset = (address & 0x2) >> 1;
                int32_t halfwordMask = ~(0xFFFF << (halfwordOffset * 16));
                int32_t newData = (wordData & halfwordMask) |
                                   ((writeData & 0xFFFF) << (halfwordOffset * 16));
                dataMemory->setMemoryValue(address & ~0x3, newData);
            }
            break;

            default:
                std::cerr << "Unknown memory write operation for opcode: 0x" <<
                        std::hex << static_cast<int>(opcode) << std::dec << std::endl;
                dataMemory->setMemoryValue(address, writeData); // Default to word write
        }
        if constexpr (DEBUG) {
            std::cout << std::endl << "CPU Memory Access: ";
            if (pipelineStructure->ex_mem.memRead) {
                std::cout << "Read data = 0x" << std::hex << memoryData << std::dec
                        << " from address 0x" << std::hex << address << std::dec;
            } else if (pipelineStructure->ex_mem.memWrite) {
                std::cout << "Wrote data 0x" << std::hex << pipelineStructure->ex_mem.rt_value
                        << std::dec << " to address 0x" << std::hex << address << std::dec;
            } else {
                std::cout << "No memory operation";
            }
            std::cout << std::endl;
        }
    }

    // Update MEM/WB pipeline register
    pipelineStructure->next_mem_wb.pc = pipelineStructure->ex_mem.pc;
    pipelineStructure->next_mem_wb.instruction = pipelineStructure->ex_mem.instruction;
    pipelineStructure->next_mem_wb.rd_num = pipelineStructure->ex_mem.rd_num;
    pipelineStructure->next_mem_wb.alu_result = pipelineStructure->ex_mem.alu_result;
    pipelineStructure->next_mem_wb.memory_read_data = memoryData;
    pipelineStructure->next_mem_wb.regWrite = pipelineStructure->ex_mem.regWrite;
    pipelineStructure->next_mem_wb.memToReg = pipelineStructure->ex_mem.memToReg;
    pipelineStructure->next_mem_wb.valid = true;
    pipelineStructure->MEM_Done = true;

    if constexpr (DEBUG) {
        std::cout << "CPU Memory Access: ";
        if (pipelineStructure->ex_mem.memRead) {
            std::cout << "Read data = 0x" << std::hex << memoryData << std::dec;
        } else if (pipelineStructure->ex_mem.memWrite) {
            std::cout << "Wrote data to address 0x" << std::hex << address;
        } else {
            std::cout << "No memory operation";
        }
        std::cout << std::endl;
    }


    std::cout << "+++++++++++++++++++++++++++++++++++++" << std::endl;
}
void CPUSimulator::writeBack() const {
    // Don't proceed if MEM stage is not done
    if (!pipelineStructure->MEM_Done) {
        pipelineStructure->WB_Done = false;
        return;
    }

    // Check if the instruction in MEM/WB is valid
    if (!pipelineStructure->mem_wb.valid) {
        pipelineStructure->WB_Done = true;
        return;
    }

    std::cout << "=-===========-=======================" << std::endl;
    std::cout << "============-CPU WriteBack===========" << std::endl;
    if constexpr (DEBUG) {
        // Debug MEM/WB register values
        std::cout << "MEM/WB Stage Register Values:" << std::endl;
        std::cout << "  RegWrite: " << pipelineStructure->mem_wb.regWrite << std::endl;
        std::cout << "  RD NUM: " << pipelineStructure->mem_wb.rd_num << " (register: " << getRegisterName(
            pipelineStructure->mem_wb.rd_num) << ")" << std::endl;
        std::cout << "  MemToReg: " << pipelineStructure->mem_wb.memToReg << std::endl;
        std::cout << "  ALU Result: " << pipelineStructure->mem_wb.alu_result << std::endl;
        std::cout << "  Memory Data: " << pipelineStructure->mem_wb.memory_read_data << std::endl;
    }


    // Only write back if regWrite is true
    if (pipelineStructure->mem_wb.regWrite) {
        int32_t writeData = pipelineStructure->mem_wb.memToReg
                                 ? pipelineStructure->mem_wb.memory_read_data
                                 : pipelineStructure->mem_wb.alu_result;

        // Write to register file
        if (pipelineStructure->mem_wb.rd_num != 0) {
            // Don't write to $0
            regfile->setRegisterValue(pipelineStructure->mem_wb.rd_num, writeData);

            if constexpr (DEBUG) {
                std::cout << std::endl << "CPU WriteBack: ";
                std::cout << "Wrote " << std::hex << writeData << std::dec <<
                        " to register $" << pipelineStructure->mem_wb.rd_num <<
                        " (" << getRegisterName(pipelineStructure->mem_wb.rd_num) << ")" << std::endl;
            }
        }
    }

    pipelineStructure->WB_Done = true;
    std::cout << "+++++++++++++++++++++++++++++++++++++" << std::endl;
}

bool CPUSimulator::detectLoadUseHazard() const {
    // Only check if we have a valid instruction in ID/EX
    if (!pipelineStructure->id_ex.valid) return false;

    // Detect load-use hazard: when ID/EX stage contains a load instruction
    // and the next instruction (in IF/ID) uses the loaded value
    if (pipelineStructure->id_ex.memRead && pipelineStructure->if_id.valid) {
        int32_t rt_dest = pipelineStructure->id_ex.rt_num; // Register to be loaded into

        // Check if the next instruction uses this register
        int32_t next_rs = pipelineStructure->if_id.instruction.getRs();
        int32_t next_rt = pipelineStructure->if_id.instruction.getRt();

        // Check if this is an R-type or I-type that uses registers
        // Some opcodes like J, JAL don't use Rs or Rt
        int8_t opcode = pipelineStructure->if_id.instruction.getOpcode();
        bool usesRegisters = true;

        // Check for J-type instructions that don't use registers
        if (opcode == 0x2 || opcode == 0x3) {
            // J, JAL
            usesRegisters = false;
        }

        if (rt_dest != 0 && usesRegisters && (rt_dest == next_rs || rt_dest == next_rt)) {
            if constexpr (DEBUG) {
                std::cout << "Load-use hazard detected! Stalling pipeline." << std::endl;
            }
            pipelineStructure->stallPipeline = true; //set stall flag
            return true; //Stall the Pipeline
        }
    }

    pipelineStructure->stallPipeline = false; //Clear the Stall
    return false;
}

void CPUSimulator::dataForwarder(int32_t &aluInput1, int32_t &aluInput2) {
    // Store original values from registers
    int32_t original_rs_value = pipelineStructure->id_ex.rs_value;
    int32_t original_rt_value = pipelineStructure->id_ex.rt_value;

    // Get register numbers
    int32_t rs_num = pipelineStructure->id_ex.rs_num;
    int32_t rt_num = pipelineStructure->id_ex.rt_num;
    bool aluSrc = pipelineStructure->id_ex.aluSrc;

    // Set initial ALU inputs
    aluInput1 = original_rs_value;
    if (aluSrc) {
        // Sign extend the immediate value
        uint16_t bit = pipelineStructure->id_ex.immediate;
        int32_t sigX = (bit & 0x8000) ? (0xFFFF0000 | bit) : bit;
        aluInput2 = sigX;
    } else {
        aluInput2 = original_rt_value;
    }

    std::cout << "DATA FORWARDING DEBUG:" << std::endl;
    std::cout << "  RS_NUM: " << rs_num << " (register: " << getRegisterName(rs_num) << ")" << std::endl;
    std::cout << "  RT_NUM: " << rt_num << " (register: " << getRegisterName(rt_num) << ")" << std::endl;
    std::cout << "  ALU_SRC: " << (aluSrc ? "Immediate" : "Register") << std::endl;
    std::cout << "  Initial ALU Inputs: " << aluInput1 << ", " << aluInput2 << std::endl;

    // Check EX/MEM forwarding
    if (pipelineStructure->ex_mem.valid && pipelineStructure->ex_mem.regWrite) {
        int32_t ex_mem_dest_reg;

        if (pipelineStructure->ex_mem.regDst == 0) {
            ex_mem_dest_reg = pipelineStructure->ex_mem.rt_num;
        } else {
            ex_mem_dest_reg = pipelineStructure->ex_mem.rd_num;
        }

        std::cout << "  EX/MEM Forwarding:" << std::endl;
        std::cout << "    EX/MEM Instruction: " << pipelineStructure->ex_mem.instruction.toString() << std::endl;
        std::cout << "    EX/MEM Dest Reg: " << getRegisterName(ex_mem_dest_reg) << " Value: " << pipelineStructure->ex_mem.alu_result << std::endl;
        std::cout << "    RS_NUM: " << getRegisterName(rs_num) << std::endl;
        std::cout << "    RT_NUM: " << getRegisterName(rt_num) << std::endl;
        std::cout << "    ALU SRC: " << pipelineStructure->id_ex.aluSrc << std::endl;

        // Forward from EX/MEM to RS
        if (ex_mem_dest_reg == rs_num && rs_num != 0) {
            // Store the forwarded value in the next register
            pipelineStructure->id_ex.forwarded_rs_value = pipelineStructure->ex_mem.alu_result;
            // Update the current ALU input for this cycle
            aluInput1 = pipelineStructure->ex_mem.alu_result;
        }

        // Forward from EX/MEM to RT (only if RT is used as input, i.e., aluSrc is false)
        if (ex_mem_dest_reg == rt_num && rt_num != 0 && !aluSrc) {
            // Store the forwarded value in the next register
            pipelineStructure->id_ex.forwarded_rt_value = pipelineStructure->ex_mem.alu_result;
            // Update the current ALU input for this cycle
            aluInput2 = pipelineStructure->ex_mem.alu_result;
        }
    }

    // Check MEM/WB forwarding
    if (pipelineStructure->mem_wb.valid && pipelineStructure->mem_wb.regWrite) {
        int32_t mem_wb_dest_reg = pipelineStructure->mem_wb.write_reg_num;
        int32_t mem_wb_value;

        if (pipelineStructure->mem_wb.memToReg) {
            mem_wb_value = pipelineStructure->mem_wb.memory_read_data;
        } else {
            mem_wb_value = pipelineStructure->mem_wb.alu_result;
        }

        std::cout << "  MEM/WB Forwarding:" << std::endl;
        std::cout << "    MEM/WB Instruction: " << pipelineStructure->mem_wb.instruction.toString() << std::endl;
        std::cout << "    MEM/WB Dest Reg: " << getRegisterName(mem_wb_dest_reg) << " Value: " << mem_wb_value << std::endl;
        std::cout << "    RS_NUM: " << getRegisterName(rs_num) << std::endl;
        std::cout << "    RT_NUM: " << getRegisterName(rt_num) << std::endl;
        std::cout << "    ALU SRC: " << pipelineStructure->id_ex.aluSrc << std::endl;

        // Only forward from MEM/WB if EX/MEM didn't already forward
        // Forward from MEM/WB to RS
        if (mem_wb_dest_reg == rs_num && rs_num != 0 && !(pipelineStructure->ex_mem.valid && pipelineStructure->ex_mem.regWrite &&
            (pipelineStructure->ex_mem.rd_num == rs_num || pipelineStructure->ex_mem.rt_num == rs_num))) {
            // Store the forwarded value in the next register
            pipelineStructure->id_ex.forwarded_rs_value = mem_wb_value;
            // Update the current ALU input for this cycle
            aluInput1 = mem_wb_value;
        }

        // Forward from MEM/WB to RT (only if RT is used as input)
        if (mem_wb_dest_reg == rt_num && rt_num != 0 && !aluSrc && !(pipelineStructure->ex_mem.valid && pipelineStructure->ex_mem.regWrite &&
            (pipelineStructure->ex_mem.rd_num == rt_num || pipelineStructure->ex_mem.rt_num == rt_num))) {
            // Store the forwarded value in the next register
            pipelineStructure->id_ex.forwarded_rt_value = mem_wb_value;
            // Update the current ALU input for this cycle
            aluInput2 = mem_wb_value;
        }
    }

    std::cout << "  Final ALU Inputs: " << aluInput1 << ", " << aluInput2 << std::endl;
}

void CPUSimulator::startCPU() {
    if constexpr (DEBUG) {
        std::cout << "\nStarting CPU" << std::endl;
    }

    cpuRunning = true;
    cyclesExecuted = 0;

    // Initial pipeline state - all stages empty
    pipelineStructure->if_id.valid = false;
    pipelineStructure->id_ex.valid = false;
    pipelineStructure->ex_mem.valid = false;
    pipelineStructure->mem_wb.valid = false;
    pipelineStructure->IF_Done = false;
    pipelineStructure->ID_Done = false;
    pipelineStructure->EX_Done = false;
    pipelineStructure->MEM_Done = false;
    pipelineStructure->WB_Done = false;

    while (cpuRunning) {
        cyclesExecuted++;
        std::cout <<
                "CYCLESTARTCYCLESTARTCYCLESTARTCYCLESTARTCYCLESTARTCYCLESTARTCYCLESTARTCYCLESTARTCYCLESTARTCYCLESTARTCYCLESTARTCYCLESTARTCYCLESTARTCYCLESTARTCYCLESTART"
                << std::endl;
        // Execute pipeline stages in reverse order to avoid overwriting data
        // Stages that depend on previous stages should go first
        writeBack(); // Stage 5 - doesn't depend on other stages in the current cycle
        memoryAccess();
        execute();
        decode();
        fetch();
        updatePipelineRegisters();
        programCounter->updatePC();
        virtualClock();

        if constexpr (DEBUG) {
            printf("Cycle: %d, PC: 0x%x\n", cyclesExecuted, programCounter->getPC());
            printPipelineState();
            std::cout << "Stage flags: IF=" << pipelineStructure->IF_Done
                      << " ID=" << pipelineStructure->ID_Done
                      << " EX=" << pipelineStructure->EX_Done
                      << " MEM=" << pipelineStructure->MEM_Done
                      << " WB=" << pipelineStructure->WB_Done << std::endl;
        }

        // Termination conditions
        bool noMoreInstructions = programCounter->getPC() >= instructionMemory->getMemory().size() * 4;
        bool pipelineEmpty = !pipelineStructure->if_id.valid &&
                             !pipelineStructure->id_ex.valid &&
                             !pipelineStructure->ex_mem.valid &&
                             !pipelineStructure->mem_wb.valid;

        if (noMoreInstructions && pipelineEmpty) {
            std::cout << "Program completed. All instructions processed. CPU stopping." << std::endl;
            cpuRunning = false;
        }

        std::cout << "CYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEEND" << std::endl;
    }
}

void CPUSimulator::virtualClock() {
    //https://stackoverflow.com/questions/158585/how-do-you-add-a-timed-delay-to-a-c-program
    // using namespace std::this_thread; // sleep_for, sleep_until
    // using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
    // using std::chrono::system_clock;
    clock = true;
    if (clock) {
        if constexpr (DEBUG) {
            std::cout << "Positive clock edge" << std::endl;
        }



        clock = false;
        if (!clock) {
            if constexpr (DEBUG) {
                std::cout << "Negative clock edge" << std::endl;
            }
        }
    }
}

void CPUSimulator::printPipelineState() const {
    std::cout << "Pipeline State:" << std::endl;
    std::cout << "  IF/ID: " << (pipelineStructure->if_id.valid
                                     ? pipelineStructure->if_id.instruction.toString()
                                     : "NOP") << std::endl;
    std::cout << "  ID/EX: " << (pipelineStructure->id_ex.valid
                                     ? pipelineStructure->id_ex.instruction.toString()
                                     : "NOP") << std::endl;
    std::cout << "  EX/MEM: " << (pipelineStructure->ex_mem.valid
                                      ? pipelineStructure->ex_mem.instruction.toString()
                                      : "NOP") << std::endl;
    std::cout << "  MEM/WB: " << (pipelineStructure->mem_wb.valid
                                      ? pipelineStructure->mem_wb.instruction.toString()
                                      : "NOP") << std::endl;
}

std::string CPUSimulator::getRegisterName(const int8_t regNum) {
    static const std::string regNames[] = {
        "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3",
        "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
        "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
        "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"
    };

    if (regNum < 32) {
        return regNames[regNum];
    }
    return "unknown";
}

void CPUSimulator::setControlSignals(const Instruction &instr) const {
    InstructionType type = instr.getType();
    int8_t opcode = instr.getOpcode();
    int8_t funct = instr.getFunct();

    // Default values
    pipelineStructure->id_ex.regWrite = false;
    pipelineStructure->id_ex.memRead = false;
    pipelineStructure->id_ex.memWrite = false;
    pipelineStructure->id_ex.memToReg = false;
    pipelineStructure->id_ex.branch = false;
    pipelineStructure->id_ex.jump = false;
    pipelineStructure->id_ex.aluOp = 0;
    pipelineStructure->id_ex.aluSrc = false;
    pipelineStructure->id_ex.regDst = 0;

    if (type == InstructionType::R_Instruciton) {
        // R-type instructions
        pipelineStructure->id_ex.regWrite = true;
        pipelineStructure->id_ex.regDst = 1; // Use rd field
        pipelineStructure->id_ex.aluSrc = false; // Use rt value
        pipelineStructure->id_ex.aluOp = funct; // Function code determines ALU operation

        // Special case for jr
        if (funct == InstructionSet::JR) {
            pipelineStructure->id_ex.jump = true;
            pipelineStructure->id_ex.regWrite = false;
        }
    } else if (type == InstructionType::I_Instruction) {
        // I-type instructions
        pipelineStructure->id_ex.aluSrc = true; // Use immediate value

        if (opcode == InstructionSet::LW) {
            // Load word
            pipelineStructure->id_ex.regWrite = true;
            pipelineStructure->id_ex.memRead = true;
            pipelineStructure->id_ex.memToReg = true;
            pipelineStructure->id_ex.regDst = 0; // Use rt field
        } else if (opcode == InstructionSet::SW) {
            // Store word
            pipelineStructure->id_ex.memWrite = true;
        } else if (opcode == InstructionSet::BEQ || opcode == InstructionSet::BNE) {
            // Branch
            pipelineStructure->id_ex.branch = true;
        } else {
            // Other I-type (addi, etc.)
            pipelineStructure->id_ex.regWrite = true;
            pipelineStructure->id_ex.regDst = 0; // Use rt field
        }
    }

    // Handle jump instructions
    if (opcode == InstructionSet::J) {
        pipelineStructure->id_ex.jump = true;
        pipelineStructure->id_ex.aluOp = InstructionSet::J; // Use 0x2 for J
    } else if (opcode == InstructionSet::JAL) {
        pipelineStructure->id_ex.jump = true;
        pipelineStructure->id_ex.regWrite = true; // JAL writes to $ra
        pipelineStructure->id_ex.aluOp = InstructionSet::JAL; // Use 0x3 for JAL
        pipelineStructure->id_ex.rd_num = 31;
    }
    // Handle branch instructions
    else if (opcode == InstructionSet::BEQ) {
        pipelineStructure->id_ex.branch = true;
        pipelineStructure->id_ex.aluOp = InstructionSet::BEQ; // Use 0x4 for BEQ
    } else if (opcode == InstructionSet::BNE) {
        pipelineStructure->id_ex.branch = true;
        pipelineStructure->id_ex.aluOp = InstructionSet::BNE; // Use 0x5 for BNE
    }

    if (type == InstructionType::J_Instruction) {
        // J-type instructions
        pipelineStructure->id_ex.aluOp = 0x2;
        pipelineStructure->id_ex.jump = true;

        if (opcode == InstructionSet::JAL) {
            // Jump and link
            pipelineStructure->id_ex.regWrite = true;
            // Special case: write to $ra (register 31)
            pipelineStructure->id_ex.rd_num = 31;
        }
    }
}

void CPUSimulator::updatePipelineRegisters() const {
    Instruction nopInstr(0x0);

    if constexpr (DEBUG) {
        std::cout << "\n=== PIPELINE REGISTER UPDATE ===" << std::endl;
    }

    // IF/ID Update
    if (!pipelineStructure->stallPipeline) {
        if constexpr (DEBUG) {
            std::cout << "IF/ID Update: "
                    << (pipelineStructure->next_if_id.valid
                            ? pipelineStructure->next_if_id.instruction.toString()
                            : "NOP")
                    << std::endl;
        }
        pipelineStructure->if_id.pc = pipelineStructure->next_if_id.pc;
        pipelineStructure->if_id.instruction = pipelineStructure->next_if_id.instruction;
        pipelineStructure->if_id.valid = pipelineStructure->next_if_id.valid;
    }

    // ID/EX Update
    if (!pipelineStructure->stallPipeline) {
        if constexpr (DEBUG) {
            std::cout << "ID/EX Update: "
                    << (pipelineStructure->next_id_ex.valid
                            ? pipelineStructure->next_id_ex.instruction.toString()
                            : "NOP")
                    << " regWrite=" << pipelineStructure->next_id_ex.regWrite
                    << std::endl;
        }
        // Copy all ID/EX fields individually
        pipelineStructure->id_ex.pc = pipelineStructure->next_id_ex.pc;
        pipelineStructure->id_ex.instruction = pipelineStructure->next_id_ex.instruction;
        pipelineStructure->id_ex.rs_num = pipelineStructure->next_id_ex.rs_num;
        pipelineStructure->id_ex.rt_num = pipelineStructure->next_id_ex.rt_num;
        pipelineStructure->id_ex.rd_num = pipelineStructure->next_id_ex.rd_num;
        pipelineStructure->id_ex.rs_value = pipelineStructure->next_id_ex.rs_value;
        pipelineStructure->id_ex.rt_value = pipelineStructure->next_id_ex.rt_value;
        pipelineStructure->id_ex.immediate = pipelineStructure->next_id_ex.immediate;
        pipelineStructure->id_ex.shamt = pipelineStructure->next_id_ex.shamt;
        pipelineStructure->id_ex.forwarded_rs_value = pipelineStructure->next_id_ex.forwarded_rs_value;
        pipelineStructure->id_ex.forwarded_rt_value = pipelineStructure->next_id_ex.forwarded_rt_value;
        // Control signals
        pipelineStructure->id_ex.regWrite = pipelineStructure->next_id_ex.regWrite;
        pipelineStructure->id_ex.memRead = pipelineStructure->next_id_ex.memRead;
        pipelineStructure->id_ex.memWrite = pipelineStructure->next_id_ex.memWrite;
        pipelineStructure->id_ex.memToReg = pipelineStructure->next_id_ex.memToReg;
        pipelineStructure->id_ex.branch = pipelineStructure->next_id_ex.branch;
        pipelineStructure->id_ex.jump = pipelineStructure->next_id_ex.jump;
        pipelineStructure->id_ex.aluOp = pipelineStructure->next_id_ex.aluOp;
        pipelineStructure->id_ex.aluSrc = pipelineStructure->next_id_ex.aluSrc;
        pipelineStructure->id_ex.regDst = pipelineStructure->next_id_ex.regDst;
        pipelineStructure->id_ex.valid = pipelineStructure->next_id_ex.valid;
    }

    // EX/MEM Update
    if (!pipelineStructure->stallPipeline) {
        if constexpr (DEBUG) {
            std::cout << "EX/MEM Update: "
                    << (pipelineStructure->next_ex_mem.valid
                            ? pipelineStructure->next_ex_mem.instruction.toString()
                            : "NOP")
                    << " alu_result=0x" << std::hex << pipelineStructure->next_ex_mem.alu_result
                    << " regWrite=" << pipelineStructure->next_ex_mem.regWrite
                    << std::dec << std::endl;
        }
        pipelineStructure->ex_mem.pc = pipelineStructure->next_ex_mem.pc;
        pipelineStructure->ex_mem.instruction = pipelineStructure->next_ex_mem.instruction;
        pipelineStructure->ex_mem.rs_num = pipelineStructure->next_ex_mem.rs_num;
        pipelineStructure->ex_mem.rt_num = pipelineStructure->next_ex_mem.rt_num;
        pipelineStructure->ex_mem.rd_num = pipelineStructure->next_ex_mem.rd_num;
        pipelineStructure->ex_mem.alu_result = pipelineStructure->next_ex_mem.alu_result;
        pipelineStructure->ex_mem.rs_value = pipelineStructure->next_ex_mem.rs_value;
        pipelineStructure->ex_mem.rt_value = pipelineStructure->next_ex_mem.rt_value;
        pipelineStructure->ex_mem.shamt = pipelineStructure->next_ex_mem.shamt;
        pipelineStructure->ex_mem.immediate = pipelineStructure->next_ex_mem.immediate;
        // Control signals
        pipelineStructure->ex_mem.regWrite = pipelineStructure->next_ex_mem.regWrite;
        pipelineStructure->ex_mem.memRead = pipelineStructure->next_ex_mem.memRead;
        pipelineStructure->ex_mem.memWrite = pipelineStructure->next_ex_mem.memWrite;
        pipelineStructure->ex_mem.memToReg = pipelineStructure->next_ex_mem.memToReg;
        pipelineStructure->ex_mem.branch = pipelineStructure->next_ex_mem.branch;
        pipelineStructure->ex_mem.jump = pipelineStructure->next_ex_mem.jump;
        pipelineStructure->ex_mem.regDst = pipelineStructure->next_ex_mem.regDst;
        pipelineStructure->ex_mem.valid = pipelineStructure->next_ex_mem.valid;
    }

    if (!pipelineStructure->stallPipeline) {
        if constexpr (DEBUG) {
            std::cout << "MEM/WB Update: "
                    << (pipelineStructure->next_mem_wb.valid
                            ? pipelineStructure->next_mem_wb.instruction.toString()
                            : "NOP")
                    << " regWrite=" << pipelineStructure->next_mem_wb.regWrite
                    << " destReg=" << pipelineStructure->next_mem_wb.write_reg_num
                    << std::endl;
        }
        pipelineStructure->mem_wb.pc = pipelineStructure->next_mem_wb.pc;
        pipelineStructure->mem_wb.instruction = pipelineStructure->next_mem_wb.instruction;
        pipelineStructure->mem_wb.rs_num = pipelineStructure->next_mem_wb.rs_num;
        pipelineStructure->mem_wb.rt_num = pipelineStructure->next_mem_wb.rt_num;
        pipelineStructure->mem_wb.rd_num = pipelineStructure->next_mem_wb.rd_num;
        pipelineStructure->mem_wb.alu_result = pipelineStructure->next_mem_wb.alu_result;
        pipelineStructure->mem_wb.memory_read_data = pipelineStructure->next_mem_wb.memory_read_data;
        pipelineStructure->mem_wb.write_data = pipelineStructure->next_mem_wb.write_data;
        pipelineStructure->mem_wb.write_reg_num = pipelineStructure->next_mem_wb.write_reg_num;
        pipelineStructure->mem_wb.rs_value = pipelineStructure->next_mem_wb.rs_value;
        pipelineStructure->mem_wb.rt_value = pipelineStructure->next_mem_wb.rt_value;
        // Control signals
        pipelineStructure->mem_wb.regWrite = pipelineStructure->next_mem_wb.regWrite;
        pipelineStructure->mem_wb.memToReg = pipelineStructure->next_mem_wb.memToReg;
        pipelineStructure->mem_wb.valid = pipelineStructure->next_mem_wb.valid;
    }

    // Handle pipeline flushes
    if (pipelineStructure->flushPipeline) {
        if constexpr (DEBUG) {
            std::cout << "FLUSHING PIPELINE" << std::endl;
        }
        pipelineStructure->if_id.instruction = nopInstr;
        pipelineStructure->if_id.valid = false;
        pipelineStructure->flushPipeline = false;
    }

    // Reset next stage registers with debug
    if constexpr (DEBUG) {
        std::cout << "Resetting next stage registers to NOP" << std::endl;
    }

    pipelineStructure->next_if_id.instruction = nopInstr;
    pipelineStructure->next_if_id.valid = false;

    pipelineStructure->next_id_ex.instruction = nopInstr;
    pipelineStructure->next_id_ex.valid = false;
    pipelineStructure->next_id_ex.regWrite = false;  // Explicitly clear control signals

    pipelineStructure->next_ex_mem.instruction = nopInstr;
    pipelineStructure->next_ex_mem.valid = false;
    pipelineStructure->next_ex_mem.regWrite = false; // Explicitly clear control signals

    pipelineStructure->next_mem_wb.instruction = nopInstr;
    pipelineStructure->next_mem_wb.valid = false;
    pipelineStructure->next_mem_wb.regWrite = false; // Explicitly clear control signals

    if constexpr (DEBUG) {
        std::cout << "=== END PIPELINE UPDATE ===\n" << std::endl;
    }
}

//Will manage cleaning pointers automativally hopefully
//(according to the new cpp standards i think)
