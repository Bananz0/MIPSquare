//
// Created by glenm on 3/22/2025.
//

// RegisterFile.h
#ifndef REGISTERFILE_H
#define REGISTERFILE_H

#include "Register.h"
#include <vector>
#include <Configureation.h>
#include <cstdint>

class RegisterFile {
public:
    RegisterFile();

    int read(RegisterNumber regNum) const;

    void write(RegisterNumber regNum, int value);

    Register &getRegister(RegisterNumber regNum);

    [[nodiscard]] uint32_t getRegisterValue(uint32_t regNum) const;

    void setRegisterValue(uint32_t regNum, uint32_t value);

private:
    std::vector<Register> registers;

};

#endif // REGISTERFILE_H
