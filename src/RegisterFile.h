//
// Created by glenm on 3/22/2025.
//

// RegisterFile.h
#ifndef REGISTERFILE_H
#define REGISTERFILE_H

#include "Register.h"
#include <vector>

class RegisterFile {
public:
    RegisterFile();
    int read(RegisterNumber regNum) const;
    void write(RegisterNumber regNum, int value);

    Register& getRegister(RegisterNumber regNum);
private:
    std::vector<Register> registers;

};

#endif // REGISTERFILE_H