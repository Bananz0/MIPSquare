//
// Created by glenm on 4/16/2025.
//

#include "CpuSimulator.h"

#include "MipsParser.h"

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
    std::vector<uint32_t> MIPSProgram;
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

void CPUSimulator::loadProgramInstructions(const std::vector<uint32_t> &memData) {
    if constexpr (DEBUG) {
        std::cout << std::endl << "Pushing Instrictions to instructionMemory" << std::endl;
    }
    instructionMemory->setMemory(memData);
    programLoaded = true;
}

void CPUSimulator::printInstructions(const std::vector<uint32_t> &instructionVector) {
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
        pipelineStructure->IF_Done = false;
        return;
    }

    // Get current PC value
    uint32_t currentPC = programCounter->getPC();

    // Check if we've reached the end of instruction memory
    if (currentPC >= instructionMemory->getMemory().size() * 4) {
        pipelineStructure->if_id.valid = false; // No more valid instructions
        pipelineStructure->IF_Done = true;
        return;
    }

    std::cout << "=========================== ====" << std::endl;
    std::cout << "=========== CPU Fetch  =========" << std::endl;

    // Fetch instruction from memory
    uint32_t instructionFetch = instructionMemory->getMemoryValue(currentPC);


    // Update IF/ID pipeline register
    pipelineStructure->if_id.pc = currentPC;
    pipelineStructure->if_id.instruction = Instruction(instructionFetch);
    pipelineStructure->if_id.valid = true;

    if constexpr (DEBUG) {
        std::cout << "CPU Fetching instruction at PC: 0x" << std::hex << currentPC << std::dec
                << std::endl;
        std::cout << pipelineStructure->if_id.instruction.toString() << std::endl;
    }

    programCounter->incrementPC();
    pipelineStructure->IF_Done = true;
    std::cout << "+++++++++++++++++++++++++++++++++++++++" << std::endl;
}

void CPUSimulator::handleBranchHazard(bool taken, uint32_t target_pc) const {
    if (taken) {
        //Update PC to branch target
        programCounter->setPC(target_pc);

        //Flush pipeline stages
        pipelineStructure->if_id.valid = false; // Invalidate IF/ID
        pipelineStructure->id_ex.valid = false; // Also invalidate ID/EX for proper flush

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
        pipelineStructure->id_ex.valid = false; //If not, pass a bubble (NOP) downstream
        pipelineStructure->ID_Done = true;
        return;
    }

    std::cout << "==========   =====================" << std::endl;
    std::cout << "===========    DECODE  ===========" << std::endl;

    // Extract instruction fields
    Instruction &instr = pipelineStructure->if_id.instruction;
    uint32_t rs_value = regfile->getRegisterValue(instr.getRs());
    uint32_t rt_value = regfile->getRegisterValue(instr.getRt());

    if constexpr (DEBUG) {
        std::cout << "\nCPU Decoding: " << instr.toString() << std::endl;
        std::cout << "Decode: reading from $" << getRegisterName(instr.getRs()) << std::endl;
        std::cout << "Decode: reading from $" << getRegisterName(instr.getRt()) << std::endl;
    }

    pipelineStructure->id_ex = pipelineStructure->if_id;


    // Initialize all the values
    uint8_t aluOp = 0;
    bool aluSrc = false;
    bool regWrite = false;
    bool memRead = false;
    bool memWrite = false;
    bool memToReg = false;
    bool branch = false;
    bool jump = false;
    uint8_t regDst = 0;

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


    //Updating ID/EX register
    pipelineStructure->id_ex.pc = pipelineStructure->if_id.pc;
    pipelineStructure->id_ex.instruction = instr;
    pipelineStructure->id_ex.rs_num = instr.getRs();
    pipelineStructure->id_ex.rt_num = instr.getRt();
    pipelineStructure->id_ex.rd_num = instr.getRd();
    pipelineStructure->id_ex.rs_value = rs_value;
    pipelineStructure->id_ex.rt_value = rt_value;
    pipelineStructure->id_ex.immediate = instr.getImmediate();
    pipelineStructure->id_ex.shamt = instr.getShamt();
    //Control Signals
    pipelineStructure->id_ex.memToReg = memToReg;
    pipelineStructure->id_ex.branch = branch;
    pipelineStructure->id_ex.jump = jump;
    pipelineStructure->id_ex.aluOp = aluOp;
    pipelineStructure->id_ex.aluSrc = aluSrc;
    pipelineStructure->id_ex.regWrite = regWrite;
    pipelineStructure->id_ex.memRead = memRead;
    pipelineStructure->id_ex.memWrite = memWrite;
    pipelineStructure->id_ex.regDst = regDst;
    pipelineStructure->id_ex.valid = true;

    pipelineStructure->ID_Done = true;

    if constexpr (DEBUG) {
        std::cout << "Decode complete - instruction moving to ID/EX stage" << std::endl;
    }
    std::cout << "+++++++++++++++++++++++++++++++++++++" << std::endl;
}

void CPUSimulator::execute() {
    // Only process if there's a valid instruction in ID/EX
    if (!pipelineStructure->id_ex.valid) {
        pipelineStructure->ex_mem.valid = false;
        pipelineStructure->EX_Done = true;
        return;
    }
    std::cout << "============================ == =" << std::endl;
    std::cout << "===========CPU EXECUTE===========" << std::endl;

    if (pipelineStructure->stallPipeline && !pipelineStructure->id_ex.memRead) {
        if constexpr (DEBUG) {
            std::cout << "Warning: Non-load instruction in execute during stall" << std::endl;
        }
    }

    uint32_t destReg = alu->getDestinationRegister(
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
    uint32_t aluInput1 = pipelineStructure->id_ex.rs_value;
    uint32_t aluInput2;

    // For shift instructions, use shamt field
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
        aluInput2 = pipelineStructure->id_ex.aluSrc
                        ? pipelineStructure->id_ex.immediate
                        : pipelineStructure->id_ex.rt_value;
    }

    // Apply forwarding logic
    dataForwarder(aluInput1, aluInput2);

    // Execute ALU operation
    bool branchTaken = false;
    uint32_t aluResult = alu->execute(
        pipelineStructure->id_ex.aluOp,
        aluInput1,
        aluInput2,
        pipelineStructure->id_ex.pc,
        branchTaken
    );

    if constexpr (DEBUG) {
        std::cout << "ALU Result: " << std::hex << aluResult << std::dec << std::endl;
    }

    // Update EX/MEM pipeline register
    pipelineStructure->ex_mem.pc = pipelineStructure->id_ex.pc;
    pipelineStructure->ex_mem.instruction = pipelineStructure->id_ex.instruction;
    pipelineStructure->ex_mem.rs_num = pipelineStructure->id_ex.rs_num;
    pipelineStructure->ex_mem.rt_num = pipelineStructure->id_ex.rt_num;
    pipelineStructure->ex_mem.rd_num = destReg; // Use calculated destination register
    pipelineStructure->ex_mem.alu_result = aluResult;
    pipelineStructure->ex_mem.rs_value = aluInput1; // Store forwarded values
    pipelineStructure->ex_mem.rt_value = (pipelineStructure->id_ex.aluSrc)
                                             ? pipelineStructure->id_ex.rt_value
                                             : // Original rt value for store
                                             aluInput2; // Forwarded rt value
    pipelineStructure->ex_mem.immediate = pipelineStructure->id_ex.immediate;

    // Pass control signals
    pipelineStructure->ex_mem.regWrite = pipelineStructure->id_ex.regWrite;
    pipelineStructure->ex_mem.memRead = pipelineStructure->id_ex.memRead;
    pipelineStructure->ex_mem.memWrite = pipelineStructure->id_ex.memWrite;
    pipelineStructure->ex_mem.memToReg = pipelineStructure->id_ex.memToReg;
    pipelineStructure->ex_mem.jump = pipelineStructure->id_ex.jump;
    pipelineStructure->ex_mem.valid = true;

    // Handle branch instructions
    if (pipelineStructure->id_ex.branch) {
        // For BEQ, the branchTaken flag is based on aluResult == 0
        if (pipelineStructure->id_ex.aluOp == 0x1) {
            branchTaken = (aluResult == 0);
        }

        if (branchTaken) {
            // Calculate branch target
            uint32_t branchTarget = pipelineStructure->id_ex.pc + 4 +
                                    (pipelineStructure->id_ex.immediate << 2);
            handleBranchHazard(true, branchTarget);
        }
    }

    // Handle jump instructions
    if (pipelineStructure->id_ex.jump) {
        uint32_t jumpTarget;

        if (pipelineStructure->id_ex.aluOp == 0xF) {
            // JR
            jumpTarget = aluInput1; // Jump to address in register
        } else if (pipelineStructure->id_ex.aluOp == 0x12 || // J
                   pipelineStructure->id_ex.aluOp == 0x13) {
            // JAL
            // Jump target from J-type format: PC[31:28] | (immediate << 2)
            uint32_t pcHigh4 = pipelineStructure->id_ex.pc & 0xF0000000;
            uint32_t targetOffset = pipelineStructure->id_ex.instruction.getJumpTarget() << 2;
            jumpTarget = pcHigh4 | targetOffset;
        } else {
            jumpTarget = pipelineStructure->id_ex.pc + 4; // Default fallback
        }

        handleBranchHazard(true, jumpTarget); // Use branch hazard handler for jumps too
    }

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
        pipelineStructure->mem_wb.valid = false;
        pipelineStructure->MEM_Done = true;
        return;
    }

    std::cout << "=-===================================" << std::endl;
    std::cout << "=========CPU Memory Access+++++++++++" << std::endl;

    uint32_t memoryData = 0;
    uint32_t address = pipelineStructure->ex_mem.alu_result;
    uint8_t opcode = pipelineStructure->ex_mem.instruction.getOpcode();

    // Memory read operations
    if (pipelineStructure->ex_mem.memRead) {
        switch (opcode) {
            case 0x23: // LW
                memoryData = dataMemory->getMemoryValue(address);
                break;

            case 0x20: // LB (Load Byte)
            {
                uint32_t wordData = dataMemory->getMemoryValue(address & ~0x3); // Align to word boundary
                uint8_t byteOffset = address & 0x3; // Get byte offset within word
                uint8_t byteData = (wordData >> (byteOffset * 8)) & 0xFF;
                // Sign extend
                memoryData = (byteData & 0x80) ? (0xFFFFFF00 | byteData) : byteData;
            }
            break;

            case 0x24: // LBU (Load Byte Unsigned)
            {
                uint32_t wordData = dataMemory->getMemoryValue(address & ~0x3);
                uint8_t byteOffset = address & 0x3;
                memoryData = (wordData >> (byteOffset * 8)) & 0xFF; // No sign extension
            }
            break;

            case 0x21: // LH (Load Halfword)
            {
                uint32_t wordData = dataMemory->getMemoryValue(address & ~0x3);
                uint8_t halfwordOffset = (address & 0x2) >> 1; // 0 for lower halfword, 1 for upper
                uint16_t halfwordData = (wordData >> (halfwordOffset * 16)) & 0xFFFF;
                // Sign extend
                memoryData = (halfwordData & 0x8000) ? (0xFFFF0000 | halfwordData) : halfwordData;
            }
            break;

            case 0x25: // LHU (Load Halfword Unsigned)
            {
                uint32_t wordData = dataMemory->getMemoryValue(address & ~0x3);
                uint8_t halfwordOffset = (address & 0x2) >> 1;
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
        uint32_t writeData = pipelineStructure->ex_mem.rt_value;

        switch (opcode) {
            case 0x2B: // SW
                dataMemory->setMemoryValue(address, writeData);
                break;

            case 0x28: // SB (Store Byte)
            {
                uint32_t wordData = dataMemory->getMemoryValue(address & ~0x3);
                uint8_t byteOffset = address & 0x3;
                uint32_t byteMask = ~(0xFF << (byteOffset * 8));
                uint32_t newData = (wordData & byteMask) |
                                   ((writeData & 0xFF) << (byteOffset * 8));
                dataMemory->setMemoryValue(address & ~0x3, newData);
            }
            break;

            case 0x29: // SH (Store Halfword)
            {
                uint32_t wordData = dataMemory->getMemoryValue(address & ~0x3);
                uint8_t halfwordOffset = (address & 0x2) >> 1;
                uint32_t halfwordMask = ~(0xFFFF << (halfwordOffset * 16));
                uint32_t newData = (wordData & halfwordMask) |
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
    pipelineStructure->mem_wb.pc = pipelineStructure->ex_mem.pc;
    pipelineStructure->mem_wb.instruction = pipelineStructure->ex_mem.instruction;
    pipelineStructure->mem_wb.rd_num = pipelineStructure->ex_mem.rd_num;
    pipelineStructure->mem_wb.alu_result = pipelineStructure->ex_mem.alu_result;
    pipelineStructure->mem_wb.memory_read_data = memoryData;
    pipelineStructure->mem_wb.regWrite = pipelineStructure->ex_mem.regWrite;
    pipelineStructure->mem_wb.memToReg = pipelineStructure->ex_mem.memToReg;
    pipelineStructure->mem_wb.valid = true;

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

    pipelineStructure->MEM_Done = true;
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
        uint32_t writeData = pipelineStructure->mem_wb.memToReg
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
        uint32_t rt_dest = pipelineStructure->id_ex.rt_num; // Register to be loaded into

        // Check if the next instruction uses this register
        uint32_t next_rs = pipelineStructure->if_id.instruction.getRs();
        uint32_t next_rt = pipelineStructure->if_id.instruction.getRt();

        // Check if this is an R-type or I-type that uses registers
        // Some opcodes like J, JAL don't use Rs or Rt
        uint8_t opcode = pipelineStructure->if_id.instruction.getOpcode();
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

void CPUSimulator::dataForwarder(uint32_t &aluInput1, uint32_t &aluInput2) {
    // Get the actual source registers from ID/EX stage
    uint32_t rs_num = pipelineStructure->id_ex.rs_num;
    uint32_t rt_num = pipelineStructure->id_ex.rt_num;
    bool aluSrc = pipelineStructure->id_ex.aluSrc;

    std::cout << "DATA FORWARDING DEBUG:" << std::endl;
    std::cout << "  RS_NUM: " << rs_num << " (register: " << getRegisterName(rs_num) << ")" << std::endl;
    std::cout << "  RT_NUM: " << rt_num << " (register: " << getRegisterName(rt_num) << ")" << std::endl;
    std::cout << "  ALU_SRC: " << (aluSrc ? "Immediate" : "Register") << std::endl;
    std::cout << "  Initial ALU Inputs: " << aluInput1 << ", " << aluInput2 << std::endl;

    // Forward from MEM/WB stage first
    if (pipelineStructure->mem_wb.valid && pipelineStructure->mem_wb.regWrite) {
        uint32_t wb_data = pipelineStructure->mem_wb.memToReg
                               ? pipelineStructure->mem_wb.memory_read_data
                               : pipelineStructure->mem_wb.alu_result;

        std::cout << "  MEM/WB Instruction: " << pipelineStructure->mem_wb.instruction.toString() << std::endl;
        std::cout << "  MEM/WB: Valid=1, RegWrite=1, Target Register="
                << getRegisterName(pipelineStructure->mem_wb.rd_num)
                << " (value: " << wb_data << ")" << std::endl;

        // Forward to RS input if we need this register's value
        if (rs_num == pipelineStructure->mem_wb.rd_num && rs_num != 0) {
            aluInput1 = wb_data;
            std::cout << "  Forwarding from MEM/WB to RS input: 0x" << std::hex << wb_data << std::dec << std::endl;
        } else {
            std::cout << "  No forwarding from MEM/WB to RS: "
                    << rs_num << " != " << pipelineStructure->mem_wb.rd_num << std::endl;
        }

        // Forward to RT input if not using immediate and we need this register's value
        if (!aluSrc && rt_num == pipelineStructure->mem_wb.rd_num && rt_num != 0) {
            aluInput2 = wb_data;
            std::cout << "  Forwarding from MEM/WB to RT input: 0x" << std::hex << wb_data << std::dec << std::endl;
        }
    }

    // Then check EX/MEM forwarding (higher priority)
    if (pipelineStructure->ex_mem.valid && pipelineStructure->ex_mem.regWrite) {
        bool can_forward = true;
        uint32_t forward_data = pipelineStructure->ex_mem.alu_result;

        // Check for load instruction
        if (pipelineStructure->ex_mem.memToReg && pipelineStructure->ex_mem.memRead) {
            can_forward = false;
            std::cout << "  EX/MEM: Cannot forward from load instruction (memory read in progress)" << std::endl;
        }

        std::cout << "  EX/MEM Instruction: " << pipelineStructure->ex_mem.instruction.toString() << std::endl;
        std::cout << "  EX/MEM: Valid=1, RegWrite=1, CanForward=" << can_forward
                << ", Target Register=" << getRegisterName(pipelineStructure->ex_mem.rd_num)
                << " (value: " << forward_data << ")" << std::endl;

        if (can_forward) {
            // Forward to RS input if we need this register's value
            if (rs_num == pipelineStructure->ex_mem.rd_num && rs_num != 0) {
                aluInput1 = forward_data;
                std::cout << "  Forwarding from EX/MEM to RS input: 0x" << std::hex << forward_data << std::dec
                        << " (overriding any MEM/WB forwarding)" << std::endl;
            }

            // Forward to RT input if not using immediate and we need this register's value
            if (!aluSrc && rt_num == pipelineStructure->ex_mem.rd_num && rt_num != 0) {
                aluInput2 = forward_data;
                std::cout << "  Forwarding from EX/MEM to RT input: 0x" << std::hex << forward_data << std::dec
                        << " (overriding any MEM/WB forwarding)" << std::endl;
            }
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

        cyclesExecuted++;
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

        //Check termination conditions
        bool noMoreInstructions = programCounter->getPC() >= instructionMemory->getMemory().size() * 4;
        bool pipelineEmpty = !pipelineStructure->if_id.valid &&
                             !pipelineStructure->id_ex.valid &&
                             !pipelineStructure->ex_mem.valid &&
                             !pipelineStructure->mem_wb.valid;

        if (noMoreInstructions && pipelineEmpty) {
            std::cout << "Program completed. All instructions processed. CPU stopping." << std::endl;
            cpuRunning = false;
        }

        std::cout <<
                "CYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEENDCYCLEEND" << std::endl;

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

std::string CPUSimulator::getRegisterName(const uint8_t regNum) {
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
    uint8_t opcode = instr.getOpcode();
    uint8_t funct = instr.getFunct();

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
    } else if (type == InstructionType::J_Instruction) {
        // J-type instructions
        pipelineStructure->id_ex.jump = true;

        if (opcode == InstructionSet::JAL) {
            // Jump and link
            pipelineStructure->id_ex.regWrite = true;
            // Special case: write to $ra (register 31)
            pipelineStructure->id_ex.rd_num = 31;
        }
    }
}


//Will manage cleaning pointers automativally hopefully
//(according to the new cpp standards i think)
