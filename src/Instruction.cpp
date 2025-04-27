//
// Created by glenm on 3/20/2025.
//

#include "Instruction.h"

Instruction::Instruction() = default;

Instruction::Instruction(const uint32_t rawInstruction) {
    rawInstructionStore = rawInstruction;
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

uint32_t Instruction::getImmediate() const {
    return static_cast<uint32_t>(this->immediate);
}

uint32_t Instruction::getJumpTarget() const {
    return this->jumpTarget;
}

uint8_t Instruction::getFunct() const {
    return this->funct;
}

uint8_t Instruction::getRs() const {
    return static_cast<uint8_t>(this->rs);
}

uint8_t Instruction::getRt() const {
    return static_cast<uint8_t>(this->rt);
}

uint8_t Instruction::getRd() const {
    return static_cast<uint8_t>(this->rd);
}

uint8_t Instruction::getShamt() const {
    return this->shamt;
}

bool Instruction::isRType() const {
    return type == InstructionType::R_Instruciton;
}

bool Instruction::isIType() const {
    return type == InstructionType::I_Instruction;
}

bool Instruction::isJType() const {
    return type == InstructionType::J_Instruction;
}

std::string Instruction::toString() const {
    std::stringstream ss;
    ss << "Instruction: ";
    if (type == InstructionType::R_Instruciton && opcode == 0 && funct == InstructionSet::NOP
    && rs == RegisterNumber::zero && rt == RegisterNumber::zero
    && rd == RegisterNumber::zero && shamt == 0) {
        ss << "NOP (No Operation)";
        return ss.str();
    }
    switch (type) {
        case InstructionType::R_Instruciton:
            ss << "R-type, Opcode: 0x" << std::hex << static_cast<int>(opcode)
                    << std::dec
                    << ", rs: " << static_cast<int>(rs)
                    << ", rt: " << static_cast<int>(rt)
                    << ", rd: " << static_cast<int>(rd)
                    << ", shamt: " << static_cast<int>(shamt)
                    << ", funct: 0x" << std::hex << static_cast<int>(funct)
                    << std::dec;
            break;

        case InstructionType::I_Instruction:
            ss << "I-type, Opcode: 0x" << std::hex << static_cast<int>(opcode)
                    << std::dec
                    << ", rs: " << static_cast<int>(rs)
                    << ", rt: " << static_cast<int>(rt)
                    << ", immediate: " << immediate;
            break;

        case InstructionType::J_Instruction:
            ss << "J-type, Opcode: 0x" << std::hex << static_cast<int>(opcode)
                    << ", jumpTarget: 0x" << jumpTarget
               << std::dec;
            break;

        case InstructionType::Uninitialized:
            ss << "Instruction is Uninitialized";
            break;
    }
    return ss.str();
}

uint32_t Instruction::getRawInstruction() const {
    return rawInstructionStore;
}

//This should get the instruction from the ID as a variable and decode it to the registerfile
void Instruction::parseRawInstruction(const uint32_t raw) {
    // // Extract fields from the raw instruction
    // opcode = (raw >> 26) & 0x3F; // Bits 31-26
    // rs = static_cast<RegisterNumber>((raw >> 21) & 0x1F); // Bits 25-21
    // rt = static_cast<RegisterNumber>((raw >> 16) & 0x1F); // Bits 20-16
    // rd = static_cast<RegisterNumber>((raw >> 11) & 0x1F); // Bits 15-11
    // shamt = (raw >> 6) & 0x1F; // Bits 10-6
    // funct = raw & 0x3F; // Bits 5-0
    // immediate = raw & 0xFFFF; // Bits 15-0 (for I-type)
    // jumpTarget = raw & 0x3FFFFFF; // Bits 25-0 (for J-type)
    //
    // // Determine instruction type
    // if (opcode == 0) {
    //     type = InstructionType::R_Instruciton;
    // } else if (opcode == 2 || opcode == 3) {
    //     type = InstructionType::J_Instruction;
    // } else {
    //     type = InstructionType::I_Instruction;
    // }

    //Refractored for better parsing
    opcode = (raw >> 26) & 0x3F; //Extract opcode first

    rs = rt = rd = RegisterNumber::zero; //Set all to zero
    shamt = funct = immediate = jumpTarget = 0;

    // Check for NOP instruction (all zeros)
    if (raw == 0) {
        type = InstructionType::R_Instruciton;
        funct = InstructionSet::NOP;
        return;
    }

    //Determine type and extract relevant fields
    if (opcode == 0) {
        type = InstructionType::R_Instruciton;
        rs = static_cast<RegisterNumber>((raw >> 21) & 0x1F);
        rt = static_cast<RegisterNumber>((raw >> 16) & 0x1F);
        rd = static_cast<RegisterNumber>((raw >> 11) & 0x1F);
        shamt = (raw >> 6) & 0x1F;
        funct = raw & 0x3F;
    } else if (opcode == 2 || opcode == 3) {
        type = InstructionType::J_Instruction;
        jumpTarget = raw & 0x03FFFFFF; // 26-bit target
    } else if (opcode == 0x0C) {
        // SLLADDI
        type = InstructionType::I_Instruction;
        rs = static_cast<RegisterNumber>((raw >> 21) & 0x1F); // Source register
        rt = static_cast<RegisterNumber>((raw >> 16) & 0x1F); // Destination register
        immediate = raw & 0xFFFF;
    } else {
        type = InstructionType::I_Instruction;
        rs = static_cast<RegisterNumber>((raw >> 21) & 0x1F);
        rt = static_cast<RegisterNumber>((raw >> 16) & 0x1F);
        immediate = raw & 0xFFFF; // 16-bit immediate
    }


    if constexpr (INST_DEBUG) {
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
