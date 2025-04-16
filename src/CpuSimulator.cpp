//
// Created by glenm on 4/16/2025.
//

#include "CpuSimulator.h"

CPUSimulator:: CPUSimulator() {
    regfile = new RegisterFile();
    instructionMemory = new Memory();
    dataMemory = new Memory();
}

CPUSimulator::~CPUSimulator() = default;
//Will manage cleaning pointers automativally hopefully
//(according to the new cpp standards i think)
