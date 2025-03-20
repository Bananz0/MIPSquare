//
// Created by glenm on 3/20/2025.
//

#ifndef REGISTER_H
#define REGISTER_H

#include <string>
#include <iostream>

class Register {
public:
    Register(int registerNumberIn);

    void setValue(int regIn); //Save the Value to the individual Register
    int getValue() const; //Get the value from teh register
    std::string getRegisterName();

private:
    int registerNumber;
    std::string registerName;
    int registerValue{};

    void setRegisterName();
};

#endif //REGISTER_H
