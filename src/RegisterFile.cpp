//
// Created by glenm on 3/22/2025.
//

#include "RegisterFile.h"

#define REGFILE_DEBUG 1

RegisterFile::RegisterFile() : registers() {
    registers.reserve(32); //Reserve the 32 registers

    for (int i = 0; i < 32; i++) {
        registers.emplace_back(static_cast<RegisterNumber>(i));
    }

    if constexpr (REGFILE_DEBUG) {
        std::cerr << "RegisterFile initialized with 32 registers\n";
    }
}

int RegisterFile::read(RegisterNumber regNum) const {
    int regPos = static_cast<int>(regNum);
    if constexpr (REGFILE_DEBUG) {
        std::cerr << "Reading from " << registers[regPos].getRegisterName() << "\n";
    }
    return registers[regPos].getValue();
}

void RegisterFile::write(RegisterNumber regNum, int value) {
    int regPos = static_cast<int>(regNum);
    if constexpr (REGFILE_DEBUG) {
        std::cerr << "Writing 0x" << std::hex << value << " to " << registers[regPos].getRegisterName() << "\n";
    }
    registers[regPos].setValue(value);
}

Register &RegisterFile::getRegister(RegisterNumber regNum) {
    int regPos = static_cast<int>(regNum);
    return registers[regPos];
}
