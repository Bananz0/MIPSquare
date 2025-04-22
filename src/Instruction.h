//
// Created by glenm on 3/20/2025.
//

#ifndef INSTRUCTION_H
#define INSTRUCTION_H
#include <cstdint>
#include <string>
#include <Configureation.h>
#include <iomanip>
#include <bitset>

#include "Register.h"

namespace InstructionSet {
    //R Type Instructions Definitoins
    constexpr uint8_t ADD = 0x20; //0b100000
    constexpr uint8_t SUB = 0x22; //0b100010
    constexpr uint8_t AND = 0x24; //0b100100
    constexpr uint8_t OR = 0x25; //0b100101
    constexpr uint8_t SLT = 0x2A; //0b101010
    constexpr uint8_t SLL = 0x00; //0b000000
    constexpr uint8_t SRL = 0x02; //0b000010
    constexpr uint8_t SRA = 0x03; //0b000011
    constexpr uint8_t JR = 0x08; //0b001000

    //I Type Instructions Definitions
    constexpr uint8_t ADDI = 0x08; //0b001000
    constexpr uint8_t LW = 0x23; //0b100011
    constexpr uint8_t SW = 0x2B; //0b101011
    constexpr uint8_t BEQ = 0x04; //0b000100
    constexpr uint8_t BNE = 0x05; //0b000101

    //J Type Instruction Definitions
    constexpr uint8_t J = 0x02; //0b000010
    constexpr uint8_t JAL = 0x03; //0b000011
}

enum class InstructionType {
    R_Instruciton, I_Instruction, J_Instruction, Uninitialized
};

class Instruction {
public:
    Instruction();
    explicit Instruction(uint32_t rawInstruction);
    explicit Instruction(const std::string& asmInstruction);

    [[nodiscard]] InstructionType getType() const;
    [[nodiscard]] uint8_t getOpcode() const;
    [[nodiscard]] RegisterNumber getSourceReg1() const;
    [[nodiscard]] RegisterNumber getSourceReg2() const;
    [[nodiscard]] RegisterNumber getDestReg() const;
    [[nodiscard]] uint16_t getImmediate() const;
    [[nodiscard]] uint32_t getJumpTarget() const;
    [[nodiscard]] uint8_t getFunct() const;

    [[nodiscard]] std::string toString() const;

    void parseRawInstruction(uint32_t raw);

    static void parseAssemblyInstruction(const std::string &asm_text);

private:
    InstructionType type = InstructionType::Uninitialized;
    uint8_t opcode = 0;
    RegisterNumber rs = RegisterNumber::a0;
    RegisterNumber rt = RegisterNumber::a0;
    RegisterNumber rd = RegisterNumber::a0;
    uint8_t shamt = 0;
    uint8_t funct = 0;
    uint16_t immediate = 0;
    uint32_t jumpTarget = 0;
};

#endif //INSTRUCTION_H
