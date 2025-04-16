//
// Created by glenm on 3/20/2025.
//

#ifndef INSTRUCTION_H
#define INSTRUCTION_H
#include <cstdint>
#include <string>
#include <Configureation.h>

#include "Register.h"

namespace InstructionSet {
    // R-type instructions
    constexpr uint8_t ADD = 0x10;
    constexpr uint8_t SUB = 0x12;
    constexpr uint8_t AND = 0x14;
    constexpr uint8_t OR = 0x16;
    constexpr uint8_t SLT = 0x18;

    // I-type instructions
    constexpr uint8_t ADDI = 0x20;
    constexpr uint8_t LW = 0x22;
    constexpr uint8_t SW = 0x24;
    constexpr uint8_t BEQ = 0x26;
    constexpr uint8_t BNE = 0x28;

    // J-type instructions
    constexpr uint8_t J = 0x69;
    constexpr uint8_t JAL = 0x42;
}

enum class InstructionType {
    R_Instruciton, I_Instruction, J_Instruction
};

class Instruction {
public:
    // Constructors
    Instruction(); // Default, creates a NOP
    explicit Instruction(uint32_t rawInstruction); // From binary
    explicit Instruction(const std::string& asmInstruction); // From assembly text

    // Instruction type and fields
    [[nodiscard]] InstructionType getType() const;
    [[nodiscard]] uint8_t getOpcode() const;
    [[nodiscard]] RegisterNumber getSourceReg1() const;
    [[nodiscard]] RegisterNumber getSourceReg2() const;
    [[nodiscard]] RegisterNumber getDestReg() const;
    [[nodiscard]] uint16_t getImmediate() const;
    [[nodiscard]] uint32_t getJumpTarget() const;

    // Debugging helpers
    [[nodiscard]] std::string toString() const;

private:
    // Instruction components
    InstructionType type;
    uint8_t opcode;
    RegisterNumber rs;
    RegisterNumber rt;
    RegisterNumber rd;
    uint8_t shamt;
    uint8_t funct;
    uint16_t immediate;
    uint32_t jumpTarget;

    // Helper methods
    void parseRawInstruction(uint32_t raw);
    void parseAssemblyInstruction(const std::string& asm_text);
};

#endif //INSTRUCTION_H
