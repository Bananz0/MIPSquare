//
// Created by glenm on 3/20/2025.
//

#ifndef INSTRUCTION_H
#define INSTRUCTION_H
#include <cstdint>
#include <string>

#include "Memory.h"
#include "Register.h"
#include "RegisterFile.h"

namespace InstructionSet {
    // R-type instructions
    constexpr uint8_t ADD = 0x20;
    constexpr uint8_t SUB = 0x22;
    constexpr uint8_t AND = 0x24;
    constexpr uint8_t OR = 0x25;
    constexpr uint8_t SLT = 0x2A;

    // I-type instructions
    constexpr uint8_t ADDI = 0x08;
    constexpr uint8_t LW = 0x23;
    constexpr uint8_t SW = 0x2B;
    constexpr uint8_t BEQ = 0x04;
    constexpr uint8_t BNE = 0x05;

    // J-type instructions
    constexpr uint8_t J = 0x02;
    constexpr uint8_t JAL = 0x03;
}

enum class InstructionType {
    R_Instruciton, I_Instruction, J_Instruction
};

class Instruction {
public:
    explicit Instruction(uint32_t encodedInstruction);
    [[nodiscard]] InstructionType getType() const;
    [[nodiscard]] uint8_t getOpcode() const;
    [[nodiscard]] RegisterNumber getRs() const;
    [[nodiscard]] RegisterNumber getRt() const;
    [[nodiscard]] RegisterNumber getRd() const;
    [[nodiscard]] uint8_t getShamt() const;
    [[nodiscard]] uint8_t getFunct() const;
    [[nodiscard]] uint16_t getImmediate() const;
    [[nodiscard]] uint32_t getJumpAddress() const;

    explicit Instruction(const std::string& instructionStr);
    void execute(RegisterFile& regFile, Memory& memory);
    [[nodiscard]] std::string toString() const;

private:
    //Instruction Details
    uint8_t opcode;//6 bits
    RegisterNumber rs;//5 bits
    RegisterNumber rt;//5 bits
    RegisterNumber rd;//5 bits
    uint8_t shamt;//5bits
    uint8_t funct;//6bits
    uint16_t immediate;//16 bits
    uint32_t jumpAddress;//26 bits

    InstructionType type;

    //For when I need to debug cause I def will need to
    void parseEncodedInstruction(uint32_t encodedInstruction);
    void parseInstructionString(const std::string& instructionStr);
};

#endif //INSTRUCTION_H
