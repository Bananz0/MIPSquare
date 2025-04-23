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

CPUSimulator::~CPUSimulator() = default;

std::vector<uint32_t> CPUSimulator::loadProgramFromFile() {
    std::vector<uint32_t> MIPSProgram;
    //Open the MIPS Program File
    std::ifstream programRaw("MIPSProgram.txt");
    if (!programRaw.is_open()) {
        std::cerr << "Error: Could not open MIPSProgram.txt\n" << std::endl;
        return {};
    }
    int myText;
    while (programRaw >> std::hex >> myText) {
        MIPSProgram.push_back(myText);
        // if constexpr (PROG_VERBOSE) {
        //     printf("Read Value (HEX): 0x%x\n", myText);
        // }
    }
    if constexpr (PROG_VERBOSE) {
        printInstructions(MIPSProgram);
    }
    //Close the file
    programRaw.close();
    return MIPSProgram;
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

void CPUSimulator::fetch() {
    // Check if pipeline is stalled
    if (detectLoadUseHazard()) {
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

    // Fetch instruction from memory
    uint32_t instructionFetch = instructionMemory->getMemoryValue(currentPC);

    // Update IF/ID pipeline register
    pipelineStructure->if_id.pc = currentPC;
    pipelineStructure->if_id.instruction = Instruction(instructionFetch);
    pipelineStructure->if_id.valid = true;

    if constexpr (DEBUG) {
        std::cout << std::endl << "CPU Fetching instruction at PC: 0x" << std::hex << currentPC << std::dec <<
                std::endl;
        std::cout << pipelineStructure->if_id.instruction.toString() << std::endl;
    }

    programCounter->incrementPC();
    pipelineStructure->IF_Done = true;
}

void CPUSimulator::handleBranchHazard(bool taken, uint32_t target_pc) {
    if (taken) {
        // Update PC to branch target
        programCounter->setPC(target_pc);

        // Flush pipeline stages
        pipelineStructure->if_id.valid = false;

        if constexpr (DEBUG) {
            std::cout << "Branch taken. Flushing pipeline and jumping to 0x" <<
                    std::hex << target_pc << std::dec << std::endl;
        }
    }
}


void CPUSimulator::decode() {
    if (!pipelineStructure->if_id.valid) {
        //Checks if the instruction in If/Id is a valid instruction
        pipelineStructure->id_ex.valid = false; //If not, pass a bubble (NOP) downstream
        pipelineStructure->ID_Done = true;
        return;
    }


    // Extract instruction fields
    Instruction &instr = pipelineStructure->if_id.instruction;
    uint32_t rs_value = regfile->getRegisterValue(instr.getRs());
    uint32_t rt_value = regfile->getRegisterValue(instr.getRt());

    if constexpr (DEBUG) {
        std::cout << "\nCPU Decoding: " << instr.toString() << std::endl;
        std::cout << "Reading from $" << getRegisterName(instr.getRs()) << std::endl;
        std::cout << "Reading from $" << getRegisterName(instr.getRt()) << std::endl;
    }

    //Initialize all the values
    uint8_t aluOp = 0;
    bool aluSrc = false;
    bool regWrite = false;
    bool memRead = false;
    bool memWrite = false;
    bool memToReg = false;
    bool branch = false;

    switch (instr.getOpcode()) {
        case 0x08: //ADDI
            aluOp = 0x0; // Add
            aluSrc = true; // Immediate value
            regWrite = true;
            break;
        case 0x23: //LW
            memRead = true;
            regWrite = true;
            memToReg = true;
            aluOp = 0x0; //Add base and offset
            aluSrc = true; //alu source is immediate
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
                    break;
                case 0x00: //SLL
                    aluOp = 0x4;
                    regWrite = true;
                    break;
            }
            break;
        case 0x25: //SLTIU
            aluOp = 0x2; //Set less than Immediate unsignd
            aluSrc = true; //Immiedate value
            regWrite = true;
            break;
        default:
            std::cout << "Invalid instruction" << std::endl;
            break;
    }
    //Updating ID/EX register
    pipelineStructure->id_ex.pc = pipelineStructure->if_id.pc;
    pipelineStructure->id_ex.instruction = instr;
    pipelineStructure->id_ex.rs_num = instr.getRs();
    pipelineStructure->id_ex.rt_num = instr.getRt();
    pipelineStructure->id_ex.rs_value = rs_value;
    pipelineStructure->id_ex.rt_value = rt_value;
    pipelineStructure->id_ex.immediate = instr.getImmediate();
    //Control Signals
    pipelineStructure->id_ex.aluOp = aluOp;
    pipelineStructure->id_ex.aluSrc = aluSrc;
    pipelineStructure->id_ex.regWrite = regWrite;
    pipelineStructure->id_ex.memRead = memRead;
    pipelineStructure->id_ex.memWrite = memWrite;
    pipelineStructure->id_ex.valid = true;

    pipelineStructure->ID_Done = true;

    if constexpr (DEBUG) {
        std::cout << "Decode complete - instruction moving to ID/EX stage" << std::endl;
    }
}

void CPUSimulator::execute() {
    // Only process if there's a valid instruction in ID/EX
    if (!pipelineStructure->id_ex.valid) {
        pipelineStructure->ex_mem.valid = false;
        pipelineStructure->EX_Done = true;
        return;
    }

    // Get ALU inputs with forwarding
    uint32_t aluInput1 = pipelineStructure->id_ex.rs_value;
    uint32_t aluInput2 = pipelineStructure->id_ex.aluSrc
                             ? pipelineStructure->id_ex.immediate
                             : pipelineStructure->id_ex.rt_value;

    // Apply forwarding logic
    dataForwarder(aluInput1, aluInput2);

    // Perform ALU operation
    uint32_t aluResult = 0;

    if constexpr (DEBUG) {
        std::cout << "\nCPU Executing: " << pipelineStructure->id_ex.instruction.toString() << std::endl;
        std::cout << "ALU Inputs: " << aluInput1 << ", " << aluInput2 << std::endl;
    }

    // ALU operations based on aluOp
    switch (pipelineStructure->id_ex.aluOp) {
        case 0x0: // Add (ADDI, LW, SW, ADD)
            aluResult = aluInput1 + aluInput2;
            if constexpr (DEBUG) std::cout << "ALU: Add operation" << std::endl;
            break;

        case 0x1: // Subtract (for branch BEQ)
            aluResult = aluInput1 - aluInput2;
            if constexpr (DEBUG) std::cout << "ALU: Subtract operation" << std::endl;
            break;

        case 0x2: // SLTIU (Set Less Than Immediate Unsigned)
            aluResult = (aluInput1 < aluInput2) ? 1 : 0;
            if constexpr (DEBUG) std::cout << "ALU: SLTIU operation" << std::endl;
            break;
        case 0x4: //SLL
            aluResult = aluInput1 << aluInput2;

        default:
            std::cerr << "Error: Unknown ALU operation: 0x" << std::hex << static_cast<int>(pipelineStructure->id_ex.
                aluOp) << std::dec << std::endl;
            break;
    }

    if constexpr (DEBUG) {
        std::cout << "ALU Result: " << aluResult << std::endl;
    }

    // Update EX/MEM pipeline register
    pipelineStructure->ex_mem.pc = pipelineStructure->id_ex.pc;
    pipelineStructure->ex_mem.instruction = pipelineStructure->id_ex.instruction;
    pipelineStructure->ex_mem.rs_num = pipelineStructure->id_ex.rs_num;
    pipelineStructure->ex_mem.rt_num = pipelineStructure->id_ex.rt_num;
    pipelineStructure->ex_mem.rd_num = pipelineStructure->id_ex.rd_num;
    pipelineStructure->ex_mem.alu_result = aluResult;
    pipelineStructure->ex_mem.rs_value = aluInput1;
    pipelineStructure->ex_mem.rt_value = pipelineStructure->id_ex.rt_value;

    // Pass control signals
    pipelineStructure->ex_mem.regWrite = pipelineStructure->id_ex.regWrite;
    pipelineStructure->ex_mem.memRead = pipelineStructure->id_ex.memRead;
    pipelineStructure->ex_mem.memWrite = pipelineStructure->id_ex.memWrite;
    pipelineStructure->ex_mem.memToReg = pipelineStructure->id_ex.memToReg;
    pipelineStructure->ex_mem.valid = true;

    // Handle branch instructions
    if (pipelineStructure->id_ex.branch) {
        bool branchTaken = (aluResult == 0);
        if (branchTaken) {
            // Calculate branch target
            uint32_t branchTarget = pipelineStructure->id_ex.pc + 4 +
                                    (pipelineStructure->id_ex.immediate << 2);
            handleBranchHazard(true, branchTarget);
        }
    }

    pipelineStructure->EX_Done = true;
}
void CPUSimulator::memoryAccess() {
    if (!pipelineStructure->EX_Done) return;

    // Check if the instruction in EX/MEM is valid
    if (!pipelineStructure->ex_mem.valid) {
        // Bubble in the pipeline
        pipelineStructure->mem_wb.valid = false;
        pipelineStructure->MEM_Done = true;
        return;
    }

    uint32_t memoryData = 0;

    // Memory read operation
    if (pipelineStructure->ex_mem.memRead) {
        memoryData = dataMemory->getMemoryValue(pipelineStructure->ex_mem.alu_result);
    }

    // Memory write operation
    if (pipelineStructure->ex_mem.memWrite) {
        dataMemory->setMemoryValue(pipelineStructure->ex_mem.alu_result,
                                   pipelineStructure->ex_mem.rt_value);
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
        std::cout << std::endl << "CPU Memory Access: ";
        if (pipelineStructure->ex_mem.memRead) {
            std::cout << "Read data = " << std::hex << memoryData << std::dec;
        } else if (pipelineStructure->ex_mem.memWrite) {
            std::cout << "Wrote data to address 0x" << std::hex << pipelineStructure->ex_mem.alu_result;
        } else {
            std::cout << "No memory operation";
        }
        std::cout << std::endl;
    }

    pipelineStructure->MEM_Done = true;
}

void CPUSimulator::writeBack() {
    if (!pipelineStructure->MEM_Done) return;

    // Check if the instruction in MEM/WB is valid
    if (!pipelineStructure->mem_wb.valid) {
        pipelineStructure->WB_Done = true;
        return;
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
                std::cout << std::endl << "CPU WriteBack: Wrote " << std::hex << writeData <<
                        " to register $" << std::dec << pipelineStructure->mem_wb.rd_num << std::endl;
            }
        }
    }

    pipelineStructure->WB_Done = true;
}

bool CPUSimulator::detectLoadUseHazard() {
    // Only check if we have a valid instruction in ID/EX
    if (!pipelineStructure->id_ex.valid) return false;

    // Detect load-use hazard: when ID/EX stage contains a load instruction
    // and the next instruction (in IF/ID) uses the loaded value
    if (pipelineStructure->id_ex.memRead) {
        uint32_t rt_dest = pipelineStructure->id_ex.rd_num; // Register to be loaded into

        // Check if the next instruction uses this register
        uint32_t next_rs = pipelineStructure->if_id.instruction.getRs();
        uint32_t next_rt = pipelineStructure->if_id.instruction.getRt();

        if (rt_dest == next_rs || rt_dest == next_rt) {
            if constexpr (DEBUG) {
                std::cout << "Load-use hazard detected! Stalling pipeline." << std::endl;
            }
            return true; // Stall needed
        }
    }

    return false;
}

void CPUSimulator::dataForwarder(uint32_t &aluInput1, uint32_t &aluInput2) {
    uint32_t rs_num = pipelineStructure->id_ex.rs_num;
    uint32_t rt_num = pipelineStructure->id_ex.rt_num;

    // Forward from EX/MEM stage
    if (pipelineStructure->ex_mem.valid && pipelineStructure->ex_mem.regWrite &&
        pipelineStructure->ex_mem.rd_num != 0) {
        // RS forwarding
        if (pipelineStructure->ex_mem.rd_num == rs_num) {
            aluInput1 = pipelineStructure->ex_mem.alu_result;
            if constexpr (DEBUG) {
                std::cout << "Forwarding from EX/MEM to RS input" << std::endl;
            }
        }

        // RT forwarding (only if RT is used as input - not for immediate)
        if (!pipelineStructure->id_ex.aluSrc && pipelineStructure->ex_mem.rd_num == rt_num) {
            aluInput2 = pipelineStructure->ex_mem.alu_result;
            if constexpr (DEBUG) {
                std::cout << "Forwarding from EX/MEM to RT input" << std::endl;
            }
        }
    }

    // Forward from MEM/WB stage
    if (pipelineStructure->mem_wb.valid && pipelineStructure->mem_wb.regWrite &&
        pipelineStructure->mem_wb.rd_num != 0) {
        uint32_t wb_data = pipelineStructure->mem_wb.memToReg
                               ? pipelineStructure->mem_wb.memory_read_data
                               : pipelineStructure->mem_wb.alu_result;

        // Only forward from MEM/WB if EX/MEM is not already forwarding
        if (!(pipelineStructure->ex_mem.valid && pipelineStructure->ex_mem.regWrite &&
              pipelineStructure->ex_mem.rd_num == rs_num) &&
            pipelineStructure->mem_wb.rd_num == rs_num) {
            aluInput1 = wb_data;
            if constexpr (DEBUG) {
                std::cout << "Forwarding from MEM/WB to RS input" << std::endl;
            }
        }

        // Only forward from MEM/WB if EX/MEM is not already forwarding
        if (!pipelineStructure->id_ex.aluSrc &&
            !(pipelineStructure->ex_mem.valid && pipelineStructure->ex_mem.regWrite &&
              pipelineStructure->ex_mem.rd_num == rt_num) &&
            pipelineStructure->mem_wb.rd_num == rt_num) {
            aluInput2 = wb_data;
            if constexpr (DEBUG) {
                std::cout << "Forwarding from MEM/WB to RT input" << std::endl;
            }
        }
    }
}

void CPUSimulator::startCPU() {
    if constexpr (DEBUG) {
        std::cout << "\nStarting CPU" << std::endl;
    }

    cpuRunning = true;
    uint32_t cycleCount = 0;

    // Initial pipeline state - all stages empty
    pipelineStructure->if_id.valid = false;
    pipelineStructure->id_ex.valid = false;
    pipelineStructure->ex_mem.valid = false;
    pipelineStructure->mem_wb.valid = false;

    while (cpuRunning) {
        // Execute pipeline stages in reverse order to avoid overwriting data
        // Stages that depend on previous stages should go first
        writeBack(); // Stage 5 - doesn't depend on other stages in the current cycle
        memoryAccess();
        execute();
        decode();
        fetch();

        pipelineStructure->IF_Done = true; //Since fetch always completes

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

    }
}

void CPUSimulator::virtualClock() {
    //https://stackoverflow.com/questions/158585/how-do-you-add-a-timed-delay-to-a-c-program
    using namespace std::this_thread; // sleep_for, sleep_until
    using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
    using std::chrono::system_clock;

    // Optional: Only use sleep in non-debug mode to speed up debugging
    if constexpr (DEBUG) {
        sleep_for(10ns);
        sleep_until(system_clock::now() + 1s);
    }

    //Clock Starts as a negative so a cycle will he low - high - low
    if (clock) {
        cyclesExecuted++;
    }
    clock = !clock;
    // clock = ~clock; This caused some issues with the toggling
    // if (clock) {
    //     nextStage = static_cast<PipelineStages>((currentStage + 1) % (5)); //Will wrap around to 0
    //     currentStage = nextStage;
    // }
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

//Will manage cleaning pointers automativally hopefully
//(according to the new cpp standards i think)
