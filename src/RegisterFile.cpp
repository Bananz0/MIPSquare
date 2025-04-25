//
// Created by glenm on 3/22/2025.
//

#include "RegisterFile.h"

#include <cstdint>

//#define REGFILE_DEBUG 1

RegisterFile::RegisterFile() : registers() {
    registers.reserve(32); // Reserve the 32 registers

    for (int i = 0; i < 32; i++) {
        registers.emplace_back(static_cast<RegisterNumber>(i));
    }

    if constexpr (REGFILE_DEBUG) {
        std::cout << "\nRegisterFile initialized\n";
    }
}

uint32_t RegisterFile::getRegisterValue(uint32_t regNum) const {
    if (regNum >= 32) {
        std::cerr << "Error: Invalid register number: " << regNum << std::endl;
        return 0;
    }
    //Debug
    if constexpr (REGFILE_DEBUG) {
        std::cout << "RegisterFile: Reading " << registers[regNum].getRegisterName() << "\n";
    }

    const auto reg = static_cast<RegisterNumber>(regNum);
    return read(reg);
}


void RegisterFile::setRegisterValue(uint32_t regNum, uint32_t value) {
    if (regNum >= 32) {
        std::cerr << "Error: Invalid register number: " << regNum << std::endl;
        return;
    }

    if (regNum == 0) {
        return;
    }

    //Debug
    if constexpr (REGFILE_DEBUG) {
        std::cout << "RegisterFile: Setting " << registers[regNum].getRegisterName() << " to 0x" << std::hex << value << "\n";
    }

    const auto reg = static_cast<RegisterNumber>(regNum);
    write(reg, value);
}

uint32_t RegisterFile::read(RegisterNumber regNum) const {
    const int regPos = static_cast<int>(regNum);
    if constexpr (REGFILE_DEBUG) {
        std::cout << "RegisterFile: reading from " << registers[regPos].getRegisterName() << "\n";
    }
    return registers[regPos].getValue();
}

void RegisterFile::write(RegisterNumber regNum, uint32_t value) {
    int regPos = static_cast<int>(regNum);
    // Check for $zero register
    if (regPos == 0) {
        return;
    }

    if constexpr (REGFILE_DEBUG) {
        std::cout << "Writing 0x" << std::hex << value << " to " << registers[regPos].getRegisterName() << "\n";
    }
    registers[regPos].setValue(value);
}

Register &RegisterFile::getRegister(RegisterNumber regNum) {
    int regPos = static_cast<int>(regNum);
    return registers[regPos];
}