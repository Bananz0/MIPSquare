//
// Created by glenm on 3/20/2025.
//

#include "Instruction.h"

Instruction::Instruction() = default;

Instruction::Instruction(const uint32_t rawInstruction) {
    parseRawInstruction(rawInstruction);
}

Instruction::Instruction(const std::string &asmInstruction) {
    parseAssemblyInstruction(asmInstruction); //Parse it directly from Editor but
    //will implement possiblly never
}

InstructionType Instruction::getType() const {
    return this->type;
}

uint8_t Instruction::getOpcode() const {
    return this->opcode;
}

RegisterNumber Instruction::getSourceReg1() const {
    return this->rs;
}

RegisterNumber Instruction::getSourceReg2() const {
    return this->rt;
}

RegisterNumber Instruction::getDestReg() const {
    return this->rd;
}

uint16_t Instruction::getImmediate() const {
    return this->immediate;
}

uint32_t Instruction::getJumpTarget() const {
    return this->jumpTarget;
}

uint8_t Instruction::getFunct() const {
    return this->funct;
}

std::string Instruction::toString() const {
    std::stringstream ss;
    ss << "Instruction: ";
    switch (type) {
        //Switch Cases are fun
        //For debugging to see if the instructions are properly debugged
        case InstructionType::R_Instruciton:
            ss << "R-type, Opcode: 0x" << std::hex << static_cast<int>(opcode)
                    << ", rs: " << static_cast<int>(rs)
                    << ", rt: " << static_cast<int>(rt)
                    << ", rd: " << static_cast<int>(rd)
                    << ", shamt: " << static_cast<int>(shamt)
                    << ", funct: 0x" << std::hex << static_cast<int>(funct) << std::dec;
            break;
        case InstructionType::I_Instruction:
            ss << "I-type, Opcode: 0x" << std::hex << static_cast<int>(opcode)
                    << ", rs: " << static_cast<int>(rs)
                    << ", rt: " << static_cast<int>(rt)
                    << ", immediate: " << immediate;
            break;
        case InstructionType::J_Instruction:
            ss << "J-type, Opcode: 0x" << std::hex << static_cast<int>(opcode)
                    << ", jumpTarget: 0x" << std::hex << jumpTarget << std::dec;
            break;
        case InstructionType::Uninitialized:
            ss << "Instruction is Uninitialized";
            break;
    }
    return ss.str();
}

//This should get the instruction from the ID as a variable and decode it to the registerfile
void Instruction::parseRawInstruction(const uint32_t raw) {
    // Extract fields from the raw instruction
    opcode = (raw >> 26) & 0x3F; // Bits 31-26
    rs = static_cast<RegisterNumber>((raw >> 21) & 0x1F); // Bits 25-21
    rt = static_cast<RegisterNumber>((raw >> 16) & 0x1F); // Bits 20-16
    rd = static_cast<RegisterNumber>((raw >> 11) & 0x1F); // Bits 15-11
    shamt = (raw >> 6) & 0x1F; // Bits 10-6
    funct = raw & 0x3F; // Bits 5-0
    immediate = raw & 0xFFFF; // Bits 15-0 (for I-type)
    jumpTarget = raw & 0x3FFFFFF; // Bits 25-0 (for J-type)

    // Determine instruction type
    if (opcode == 0) {
        type = InstructionType::R_Instruciton;
    } else if (opcode == 2 || opcode == 3) {
        type = InstructionType::J_Instruction;
    } else {
        type = InstructionType::I_Instruction;
    }

    //Special case for JR
    if (opcode == 0 && funct == InstructionSet::JR) {
        type = InstructionType::R_Instruciton;
    }

    if constexpr (DEBUG) {
        std::cout << "Raw Instruction: 0x" << std::hex << raw << std::dec << std::endl;
        std::cout << "Opcode: 0x" << std::hex << static_cast<int>(opcode) << std::dec << std::endl;
        std::cout << "rs: " << static_cast<int>(rs) << std::endl;
        std::cout << "rt: " << static_cast<int>(rt) << std::endl;
        std::cout << "rd: " << static_cast<int>(rd) << std::endl;
        std::cout << "shamt: " << static_cast<int>(shamt) << std::endl;
        std::cout << "funct: 0x" << std::hex << static_cast<int>(funct) << std::dec << std::endl;
        std::cout << "immediate: " << immediate << std::endl;
        std::cout << "jumpTarget: 0x" << std::hex << jumpTarget << std::dec << std::endl;
        std::cout << "Type: " << static_cast<int>(type) << std::endl;
    }
}

void Instruction::parseAssemblyInstruction(const std::string &asm_text) {
    //Nope. Implemeting this will take a ton of time i dont have rn but will condisder
}
