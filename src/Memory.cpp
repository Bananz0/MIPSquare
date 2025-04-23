//
// Created by glenm on 4/16/2025.
//

#include "Memory.h"
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <stdexcept>

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

void Memory::setMemory(const std::vector<uint32_t> &memDataIn) {
    memory = memDataIn;
    if constexpr (DEBUG) {
        printf("Memory Bank Set\n");
    }
}

std::vector<uint32_t> Memory::getMemory() {
    return memory;
}

void Memory::setMemoryValue(const uint32_t address, uint32_t value) {
    size_t index = address >> 2; //Divide by 4 cause its word addressed
    if (index >= memory.size()) {
        throw std::out_of_range("Memory address out of range");
    }
    if constexpr (MEMORY_DEBUG) {
        std::cout << "Memory Bank SetValue at address: " << address
                  << " (index: " << index << ") with value 0x" << std::hex << value << std::dec << "\n";
    }
    memory[address>>2] = value;
}

uint32_t Memory::getMemoryValue(const uint32_t address) const {
    size_t index = address / 4;  // Convert byte address to word index
    if (index >= memory.size()) {
        std::cerr << "Memory address out of range: 0x" << std::hex << address << std::dec << std::endl;
        return 0;
    }

    if constexpr (MEMORY_DEBUG) {
        std::cout << "Memory Bank ReadValue at address: " << address << " (index: " << index << ") = 0x"
                  << std::hex << memory[index] << std::dec << std::endl;
    }

    return memory[index];
}

