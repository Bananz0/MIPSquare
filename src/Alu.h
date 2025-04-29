//
// Created by glenm on 4/16/2025.
//


#ifndef ALU_H
#define ALU_H

#include <cstdint>
#include <iostream>
#include <Configureation.h>
#include <bitset>
#include <string>
#include <vector>


class PipelineStructure;

class ALU {
public:
    ALU();
    ~ALU();
    enum AluOp : uint8_t {
        ADD = 0x0,
        SUB = 0x1,
        AND = 0x3,
        SLL = 0x4,
        OR = 0x5,
        XOR = 0x6,
        NOR = 0x7,
        SRL = 0x8,
        SRA = 0x9,
        SLT = 0xA,
        SLTU = 0xB,
        BNE = 0xC,
        BLEZ = 0xD,
        BGTZ = 0xE,
        JR = 0xF,
        BGEZ = 0x10,
        BLTZ = 0x11,
        J = 0x12,
        JAL = 0x13,
        LUI = 0x14
    };
    static int32_t getDestinationRegister(uint8_t regDst, int32_t rt_num, int32_t rd_num);
    static int32_t execute(uint8_t aluOp, int32_t input1, int32_t input2, int32_t pc, bool &branchTaken);
};

#endif // ALU_H
