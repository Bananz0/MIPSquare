//
// Created by glenm on 4/20/2025.
//

#include "ProgramCounter.h"

ProgramCounter::ProgramCounter() : currentPc(0), nextPc(0), prevPc(0) {
    if constexpr (DEBUG) {
        std::cout << "Program Counter Initialized" << std::endl;
    }
}

ProgramCounter::~ProgramCounter() = default;

uint32_t ProgramCounter::getPC() const {
    if constexpr (DEBUG) {
        std::cout << "Curernt Program Counter" << currentPc << std::endl;
    }
    return currentPc;
}

void ProgramCounter::setPC(const uint32_t newPC) {
    if constexpr (DEBUG) {
        if constexpr (PC_VERBOSE) {
            std::cout << "Program Counter manually set to " << newPC << " from " << currentPc << std::endl;
        }
    }
    nextPc = newPC;
}

void ProgramCounter::incrementPC() {
    if constexpr (DEBUG) {
        if constexpr (PC_VERBOSE) {
            std::cout << "Program Counter Incremented by 4 bytes" << std::endl;
        }
    }
    nextPc = currentPc + 4;
}

void ProgramCounter::decrementPC() {
    if constexpr (DEBUG) {
        if constexpr (PC_VERBOSE) {
            std::cout << "Program Counter Decremented by 4 bytes" << std::endl;
        }
    }
    nextPc = currentPc - 4;
}

void ProgramCounter::addPC(const uint32_t amount) {
    if constexpr (DEBUG) {
        if constexpr (PC_VERBOSE) {
            std::cout << "Program Counter Adding by " << amount << std::endl;
        }
    }
    nextPc = currentPc + amount;
}

void ProgramCounter::subtractPC(const uint32_t amount) {
    if constexpr (DEBUG) {
        if constexpr (PC_VERBOSE) {
            std::cout << "Program Counter Subtracting by " << amount << std::endl;
        }
    }
    nextPc = currentPc - amount;
}

void ProgramCounter::jumpTo(const uint32_t newPC) {
    if constexpr (DEBUG) {
        if constexpr (PC_VERBOSE) {
            std::cout << "Program Counter Jumping to " << newPC << std::endl;
        }
    }
    nextPc = newPC;
}

void ProgramCounter::updatePC() {
    if constexpr (DEBUG) {
        if constexpr (PC_VERBOSE) {
            std::cout << "Current PC: " << currentPc << std::endl;
            std::cout << "Next PC: " << nextPc << std::endl;
            std::cout << "Prev PC: " << prevPc << std::endl;
        }
        std::cout << "Program Counter Updated" << std::endl;
    }
    prevPc = currentPc;
    currentPc = nextPc;
}
