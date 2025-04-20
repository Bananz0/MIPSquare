//
// Created by glenm on 4/16/2025.
//

#include "CpuSimulator.h"

CPUSimulator:: CPUSimulator() :
    regfile(new RegisterFile()),
    instructionMemory(new Memory()),
    dataMemory(new Memory()),
    pipelineStructure(new PipelineStructure()),
    alu(new ALU()) {

}

CPUSimulator::~CPUSimulator() = default;

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

std::vector<Instruction> CPUSimulator::parseProgram(const std::string &content) {
    return {};
}

//Will manage cleaning pointers automativally hopefully
//(according to the new cpp standards i think)
