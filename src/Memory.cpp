//
// Created by glenm on 4/16/2025.
//

#include "Memory.h"

#include <cstdint>
#include <cstdio>

Memory::Memory() {
    if constexpr (DEBUG) {
        printf("Memory Bank Initialized\n");
    }
}

Memory::~Memory() {
    if constexpr (DEBUG) {
        printf("Memory Bank Deleted\n");
    }
}

void Memory::setMemory(const std::vector<int> &memDataIn) {
    memory = memDataIn;
    if constexpr (DEBUG) {
        printf("Memory Bank Set\n");
    }
}

std::vector<int> Memory::getMemory() {
    return memory;
}

uint32_t Memory::getMemoryValue(const uint32_t address) const {
    return memory[address>>2];
}

