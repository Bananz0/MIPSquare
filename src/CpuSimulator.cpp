//
// Created by glenm on 4/16/2025.
//

#include "CpuSimulator.h"

#include <fstream>

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

std::vector<int> CPUSimulator::loadProgramFromFile() {
    std::vector<int> MIPSProgram;
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

void CPUSimulator::loadProgramInstructions(const std::vector<int>& memData) const {
    if constexpr (DEBUG) {
        std::cout << std::endl << "Pushing Instrictions to instructionMemory" << std::endl;
    }
    instructionMemory->setMemory(memData);
}

void CPUSimulator::printInstructions(const std::vector<int> &instructionVector) {
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
        std::cout << std::endl << "Starting CPU fetch" << std::endl;
    }
}

void CPUSimulator::decode() {
    if constexpr (DEBUG) {
        std::cout << std::endl << "Starting CPU decode" << std::endl;
    }
}

void CPUSimulator::execute() {
    if constexpr (DEBUG) {
        std::cout << std::endl << "Starting CPU excecte" << std::endl;
    }
}

void CPUSimulator::memoryAccess() {
    if constexpr (DEBUG) {
        std::cout << std::endl << "Starting CPU memaccess" << std::endl;
    }
}

void CPUSimulator::writeBack() {
    if constexpr (DEBUG) {
        std::cout << std::endl << "Starting CPU wb" << std::endl;
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

}

//Will manage cleaning pointers automativally hopefully
//(according to the new cpp standards i think)
