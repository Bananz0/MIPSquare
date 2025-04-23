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
    }
    std::cout << "===============================" << std::endl;
    std::cout << "===========    END  ===========" << std::endl;
    std::cout << "===============================" << std::endl;
}

void CPUSimulator::fetch() {
    if constexpr (DEBUG) {
        std::cout << std::endl << "CPU Fetching." << std::endl;
    }
}

void CPUSimulator::decode() {
    if constexpr (DEBUG) {
        std::cout << std::endl << "CPU Decoding." << std::endl;
    }
}

void CPUSimulator::execute() {
    if constexpr (DEBUG) {
        std::cout << std::endl << "CPU Executing." << std::endl;
    }
}

void CPUSimulator::memoryAccess() {
    if constexpr (DEBUG) {
        std::cout << std::endl << "CPU Memory Access." << std::endl;
    }
}

void CPUSimulator::writeBack() {
    if constexpr (DEBUG) {
        std::cout << std::endl << "CPU WriteBack." << std::endl;
    }
}

bool CPUSimulator::detectDataHazard() {
    return false;
}

bool CPUSimulator::detectControlHazard() {
    return false;
}

void CPUSimulator::handleHazard() {
}

void CPUSimulator::dataForwarder() {
}

void CPUSimulator::startCPU() {
    if constexpr (DEBUG) {
        std::cout << std::endl << "Starting CPU" << std::endl;
    }
    //Probably will implement a state machine cycling through the cycles
    //Will have an on flag and a state where the "cpu" can kill the program
    cpuRunning = true; //Final State Kills the cpu by setting this to false

    while (cpuRunning) {
        virtualClock();

        //Conditionally run the stages only if the previous is complete
 writeBack();

        memoryAccess();
        execute();
        decode();
        fetch();

        switch (currentStage) {
            case FETCH:
                fetch();
                break;
            case DECODE:
                decode();
                break;
            case EXECUTE:
                execute();
                break;
            case MEMORY_ACCESS:
                memoryAccess();
                break;
            case WRITE_BACK:
                writeBack();
                break;
            case EXIT:
                cpuRunning = false;
                break;
            default:
                std::cerr << "Invalid Stage" << std::endl;
                break;
        }


        if constexpr (DEBUG) {
            printf("Cycle: %d, PC: 0x%x\n", cyclesExecuted, programCounter->getPC());
            // std::cout << "Cycle: " << cyclesExecuted << std::dec << ", PC: 0x" << std::hex << programCounter->getPC() << std::endl;
        }

        if (programCounter->getPC() == instructionMemory->getMemory().size() * 4) {
            //That gets the size of the integers in the vector and mult by 4 to get the bytesize (power of inshallah)
            std::cout << "Program Counter: " << programCounter->getPC() << std::hex << std::endl;
            std::cout << "Instruction Memory Size: " << instructionMemory->getMemory().size() * 4 << std::hex <<
                    std::endl;
            std::cout << "CPU Stopped" << std::endl;
            cpuRunning = false;
        }

        programCounter->incrementPC();
        programCounter->updatePC();
    }
}

void CPUSimulator::virtualClock() {
    //https://stackoverflow.com/questions/158585/how-do-you-add-a-timed-delay-to-a-c-program
    using namespace std::this_thread; // sleep_for, sleep_until
    using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
    using std::chrono::system_clock;
    sleep_for(10ns);
    sleep_until(system_clock::now() + 1s);
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

//Will manage cleaning pointers automativally hopefully
//(according to the new cpp standards i think)
