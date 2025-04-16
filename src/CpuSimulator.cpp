//
// Created by glenm on 4/16/2025.
//

#include "CpuSimulator.h"

CPUSimulator:: CPUSimulator() {
    regfile = new RegisterFile();
    instructionMemory = new Memory();
    dataMemory = new Memory();
    pipelineStructure = new PipelineStructure();
}

CPUSimulator::~CPUSimulator() = default;

void CPUSimulator::fetch() {
}

void CPUSimulator::execute() {
    if constexpr (DEBUG) {
        std::cout << std::endl << "Starting CPU excecte" << std::endl;
    }
}

void CPUSimulator::decode() {
}

void CPUSimulator::memoryAccess() {
}

void CPUSimulator::writeBack() {
}

bool CPUSimulator::detectDataHazard() {
}

bool CPUSimulator::detectControlHazard() {
}

void CPUSimulator::handleHazard() {
}

void CPUSimulator::dataForwarder() {
}

std::vector<Instruction> CPUSimulator::parseProgram(const std::string &content) {
}

//Will manage cleaning pointers automativally hopefully
//(according to the new cpp standards i think)
