//
// Created by glenm on 4/16/2025.
//

#include "PipelineStructure.h"

PipelineStructure::PipelineStructure() = default;

PipelineStructure::~PipelineStructure() = default;

PipelineStructure::IF_ID_Register &PipelineStructure::IF_ID_Register::operator=(
    const PipelineStructure::IF_ID_Register &other) {
    if (this != &other) {
        pc = other.pc;
        instruction = other.instruction;
        valid = other.valid;
    }
    return *this;
}

PipelineStructure::ID_EX_Register &PipelineStructure::ID_EX_Register::operator=(
    const PipelineStructure::ID_EX_Register &other) {
    if (this != &other) {
        pc = other.pc;
        instruction = other.instruction;
        rs_num = other.rs_num;
        rt_num = other.rt_num;
        rd_num = other.rd_num;
        rs_value = other.rs_value;
        rt_value = other.rt_value;
        immediate = other.immediate;
        shamt = other.shamt;
        forwarded_rs_value = other.forwarded_rs_value;
        forwarded_rt_value = other.forwarded_rt_value;
        regWrite = other.regWrite;
        memRead = other.memRead;
        memWrite = other.memWrite;
        memToReg = other.memToReg;
        branch = other.branch;
        jump = other.jump;
        aluOp = other.aluOp;
        aluSrc = other.aluSrc;
        valid = other.valid;
        regDst = other.regDst;
    }
    return *this;
}

PipelineStructure::ID_EX_Register &PipelineStructure::ID_EX_Register::operator=(
    const PipelineStructure::IF_ID_Register &other) {
    pc = other.pc;
    instruction = other.instruction;
    valid = other.valid;
    rs_num = instruction.getRs();
    rt_num = instruction.getRt();
    rd_num = instruction.getRd();
    immediate = instruction.getImmediate();
    shamt = instruction.getShamt();
    return *this;
}


PipelineStructure::EX_MEM_Register &PipelineStructure::EX_MEM_Register::operator=(
    const PipelineStructure::EX_MEM_Register &other) {
    if (this != &other) {
        pc = other.pc;
        instruction = other.instruction;
        rs_num = other.rs_num;
        rt_num = other.rt_num;
        rd_num = other.rd_num;
        alu_result = other.alu_result;
        rs_value = other.rs_value;
        rt_value = other.rt_value;
        shamt = other.shamt;
        immediate = other.immediate;
        regWrite = other.regWrite;
        memRead = other.memRead;
        memWrite = other.memWrite;
        memToReg = other.memToReg;
        branch = other.branch;
        jump = other.jump;
        regDst = other.regDst;
        valid = other.valid;
    }
    return *this;
}

PipelineStructure::EX_MEM_Register &PipelineStructure::EX_MEM_Register::operator=(
    const PipelineStructure::ID_EX_Register &other) {
    pc = other.pc;
    instruction = other.instruction;
    rs_num = other.rs_num;
    rt_num = other.rt_num;
    rd_num = other.rd_num;
    rs_value = other.rs_value;
    rt_value = other.rt_value;
    shamt = other.shamt;
    immediate = other.immediate;
    regWrite = other.regWrite;
    memRead = other.memRead;
    memWrite = other.memWrite;
    memToReg = other.memToReg;
    branch = other.branch;
    jump = other.jump;
    regDst = other.regDst;
    valid = other.valid;
    return *this;
}

PipelineStructure::MEM_WB_Register &PipelineStructure::MEM_WB_Register::operator=(
    const PipelineStructure::MEM_WB_Register &other) {
    if (this != &other) {
        pc = other.pc;
        instruction = other.instruction;
        rs_num = other.rs_num;
        rt_num = other.rt_num;
        rd_num = other.rd_num;
        alu_result = other.alu_result;
        memory_read_data = other.memory_read_data;
        rs_value = other.rs_value;
        rt_value = other.rt_value;
        regWrite = other.regWrite;
        memToReg = other.memToReg;
        valid = other.valid;
    }
    return *this;
}

PipelineStructure::MEM_WB_Register &PipelineStructure::MEM_WB_Register::operator=(
    const PipelineStructure::EX_MEM_Register &other) {
    pc = other.pc;
    instruction = other.instruction;
    rs_num = other.rs_num;
    rt_num = other.rt_num;
    rd_num = other.rd_num;
    alu_result = other.alu_result;
    rs_value = other.rs_value;
    rt_value = other.rt_value;
    regWrite = other.regWrite;
    memToReg = other.memToReg;
    valid = other.valid;
    return *this;
}