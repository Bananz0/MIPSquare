//
// Created by glenm on 3/20/2025.
//

#ifndef REGISTER_H
#define REGISTER_H

#include <string>
#include <iostream>
#include <cassert>
#include <Configureation.h>

enum class RegisterNumber {
    zero, at, v0, v1, a0, a1, a2, a3, t0, t1, t2, t3, t4, t5, t6, t7,
    s0, s1, s2, s3, s4, s5, s6, s7, t8, t9, k0, k1, gp, sp, fp, ra
};

class Register {
public:
    explicit Register(RegisterNumber registerNumberIn);

    void setValue(int regIn); //Save the Value to the individual Register
    [[nodiscard]] int getValue() const; //Get the value from teh register
    [[nodiscard]] std::string getRegisterName() const;

private:
    RegisterNumber registerNumber;
    std::string registerName;
    int registerValue{};

    void setRegisterName();

    static const std::string registerNames[32];
};

#endif //REGISTER_H
