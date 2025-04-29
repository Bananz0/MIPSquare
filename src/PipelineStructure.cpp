//
// Created by glenm on 4/16/2025.
//

#include "PipelineStructure.h"

PipelineStructure::PipelineStructure() = default;

PipelineStructure::~PipelineStructure() = default;

PipelineStructure::IF_ID_Register &PipelineStructure::IF_ID_Register::operator=(
    const IF_ID_Register &other) {
    std::cout << "IF_ID assignment operator called\n";
    if (this != &other) {
        if constexpr (DEBUG) {
            std::cout << "IF_ID->ID_EX COPY: pc=0x" << std::hex << other.pc << std::dec
                    << " instr=" << other.instruction.toString() << "\n";
        }
        pc = other.pc;
        instruction = other.instruction;
        valid = other.valid;
    }
    return *this;
}

PipelineStructure::ID_EX_Register &PipelineStructure::ID_EX_Register::operator=(
    const ID_EX_Register &other) {
    std::cout << "ID_EX assignment operator called\n";
    if (this != &other) {
        if constexpr (DEBUG) {
            std::cout << "ID_EX COPY: pc=0x" << std::hex << other.pc << std::dec
                    << " instr=" << other.instruction.toString()
                    << " rs=" << other.rs_num << "(" << other.rs_value << ")"
                    << " rt=" << other.rt_num << "(" << other.rt_value << ")"
                    << " valid=" << other.valid << "\n";
        }
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
    const IF_ID_Register &other) {
    if constexpr (DEBUG) {
        std::cout << "IF_ID->ID_EX COPY: pc=0x" << std::hex << other.pc << std::dec
                << " instr=" << other.instruction.toString() << "\n";
    }
    pc = other.pc;
    instruction = other.instruction;
    valid = other.valid;
    rs_num = instruction.getRs();
    rt_num = instruction.getRt();
    rd_num = instruction.getRd();
    immediate = instruction.getImmediate();
    shamt = instruction.getShamt();
    forwarded_rs_value = 0;
    forwarded_rt_value = 0;
    return *this;
}


PipelineStructure::EX_MEM_Register &PipelineStructure::EX_MEM_Register::operator=(
    const EX_MEM_Register &other) {
    if (this != &other) {
        if constexpr (DEBUG) {
            std::cout << "EX_MEM COPY: pc=0x" << std::hex << other.pc << std::dec
                    << " instr=" << other.instruction.toString()
                    << " alu_result=0x" << std::hex << other.alu_result << std::dec
                    << " dest_reg=" << other.rd_num
                    << " valid=" << other.valid << "\n";
        }
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
    const ID_EX_Register &other) {
    if constexpr (DEBUG) {
        std::cout << "ID_EX->EX_MEM COPY: pc=0x" << std::hex << other.pc << std::dec
                << " instr=" << other.instruction.toString() << "\n";
    }
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
    regWrite = other.regWrite;
    memToReg = other.memToReg;
    branch = other.branch;
    jump = other.jump;
    regDst = other.regDst;
    valid = other.valid;
    alu_result = 0;
    return *this;
}

PipelineStructure::MEM_WB_Register &PipelineStructure::MEM_WB_Register::operator=(
    const MEM_WB_Register &other) {
    std::cout << "MEM_WB assignment operator called\n";
    if (this != &other) {
        if constexpr (DEBUG) {
            std::cout << "MEM_WB COPY: pc=0x" << std::hex << other.pc << std::dec
                    << " instr=" << other.instruction.toString()
                    << " alu_result=0x" << std::hex << other.alu_result << std::dec
                    << " mem_data=0x" << std::hex << other.memory_read_data << std::dec
                    << " dest_reg=" << other.rd_num
                    << " valid=" << other.valid << "\n";
        }
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
    const EX_MEM_Register &other) {
    if constexpr (DEBUG) {
        std::cout << "EX_MEM->MEM_WB: " << other.instruction.toString()
                << " regWrite=" << other.regWrite
                << " destReg=" << ((other.regDst == 0) ? other.rt_num : other.rd_num)
                << " aluResult=0x" << std::hex << other.alu_result << std::dec << "\n";
    }
    pc = other.pc;
    instruction = other.instruction;
    rs_num = other.rs_num;
    rt_num = other.rt_num;
    rd_num = other.rd_num;
    alu_result = other.alu_result;
    memory_read_data = 0;
    write_data = other.rt_value;
    write_reg_num = (other.regDst == 0) ? other.rt_num : other.rd_num;
    rs_value = other.rs_value;
    rt_value = other.rt_value;
    regWrite = other.regWrite;
    memToReg = other.memToReg;
    valid = other.valid;
    return *this;
}