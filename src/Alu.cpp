//
// Created by glenm on 4/16/2025.
//


#include "Alu.h"
#include <iostream>
#include <iomanip>

ALU::ALU() = default;

ALU::~ALU() = default;

uint32_t ALU::getDestinationRegister(uint8_t regDst, uint32_t rt_num, uint32_t rd_num) {
    uint32_t destReg;

    switch (regDst) {
        case 0: // Use rt field
            destReg = rt_num;
            break;
        case 1: // Use rd field (R-type)
            destReg = rd_num;
            break;
        case 2: // Use $ra (31) for JAL
            destReg = 31; // $ra register
            break;
        default:
            destReg = 0; // $zero as failsafe
    }

    return destReg;
}

uint32_t ALU::execute(uint8_t aluOp, uint32_t input1, uint32_t input2, uint32_t pc, bool& branchTaken) {
    uint32_t result = 0;
    branchTaken = false;

    if constexpr (DEBUG) {
        std::cout << "ALU Inputs: " << input1 << ", " << input2 << std::endl;
    }

    // Expanded ALU operations
    switch (aluOp) {
        case ADD: // Add
            result = input1 + input2;
            if constexpr (DEBUG) std::cout << "ALU: Add operation" << std::endl;
            break;
        case SUB: // Subtract
            result = input1 - input2;
            if constexpr (DEBUG) std::cout << "ALU: Subtract operation" << std::endl;
            break;
        case AND: // AND
            result = input1 & input2;
            if constexpr (DEBUG) std::cout << "ALU: AND operation" << std::endl;
            break;
        case SLL: // SLL (Shift Left Logical)
            result = input1 << input2;
            if constexpr (DEBUG) std::cout << "ALU: SLL operation" << std::endl;
            break;
        case OR: // OR
            result = input1 | input2;
            if constexpr (DEBUG) std::cout << "ALU: OR operation" << std::endl;
            break;
        case XOR: // XOR
            result = input1 ^ input2;
            if constexpr (DEBUG) std::cout << "ALU: XOR operation" << std::endl;
            break;
        case NOR: // NOR
            result = ~(input1 | input2);
            if constexpr (DEBUG) std::cout << "ALU: NOR operation" << std::endl;
            break;
        case SRL: // SRL (Shift Right Logical)
            result = input1 >> input2;
            if constexpr (DEBUG) std::cout << "ALU: SRL operation" << std::endl;
            break;
        case SRA: // SRA (Shift Right Arithmetic)
            // C++ implementation of arithmetic shift
            if (input1 & 0x80000000) {
                // If MSB is 1
                uint32_t mask = ~((1 << (32 - input2)) - 1);
                result = (input1 >> input2) | mask;
            } else {
                result = input1 >> input2;
            }
            if constexpr (DEBUG) std::cout << "ALU: SRA operation" << std::endl;
            break;
        case SLT: // SLT (Set Less Than - signed)
            result = (static_cast<int32_t>(input1) < static_cast<int32_t>(input2)) ? 1 : 0;
            if constexpr (DEBUG) std::cout << "ALU: SLT operation" << std::endl;
            break;
        case SLTU: // SLTU (Set Less Than Unsigned)
            result = (input1 < input2) ? 1 : 0;
            if constexpr (DEBUG) std::cout << "ALU: SLTU operation" << std::endl;
            break;
        case BNE: // Special flag for BNE
            result = input1 - input2;
            branchTaken = (result != 0);
            if constexpr (DEBUG) std::cout << "ALU: BNE comparison" << std::endl;
            break;
        case BLEZ: // Special flag for BLEZ
            branchTaken = (static_cast<int32_t>(input1) <= 0);
            if constexpr (DEBUG) std::cout << "ALU: BLEZ comparison" << std::endl;
            break;
        case BGTZ: // Special flag for BGTZ
            branchTaken = (static_cast<int32_t>(input1) > 0);
            if constexpr (DEBUG) std::cout << "ALU: BGTZ comparison" << std::endl;
            break;
        case JR: // Special flag for JR
            result = input1; // Pass rs value as jump target
            if constexpr (DEBUG) std::cout << "ALU: JR operation" << std::endl;
            break;
        case BGEZ: // Special flag for BGEZ
            branchTaken = (static_cast<int32_t>(input1) >= 0);
            if constexpr (DEBUG) std::cout << "ALU: BGEZ comparison" << std::endl;
            break;
        case BLTZ: // Special flag for BLTZ
            branchTaken = (static_cast<int32_t>(input1) < 0);
            if constexpr (DEBUG) std::cout << "ALU: BLTZ comparison" << std::endl;
            break;
        case J: // Special flag for J
            // Jump target calculation handled separately
            if constexpr (DEBUG) std::cout << "ALU: J operation" << std::endl;
            break;
        case JAL: // Special flag for JAL
            result = pc + 8; // Return address (PC+8)
            if constexpr (DEBUG) std::cout << "ALU: JAL operation" << std::endl;
            break;
        case LUI: // Special flag for LUI
            result = input2 << 16; // Shift immediate left by 16 bits
            if constexpr (DEBUG) std::cout << "ALU: LUI operation" << std::endl;
            break;
        default:
            std::cerr << "Error: Unknown ALU operation: 0x" << std::hex <<
                    static_cast<int>(aluOp) << std::dec << std::endl;
            break;
    }

    if constexpr (DEBUG) {
        std::cout << "ALU Result: " << std::hex << result << std::dec << std::endl;
    }

    return result;
}