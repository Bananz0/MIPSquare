//
// Created by glenm on 4/16/2025.
//

#ifndef PIPELINESTRUCTURE_H
#define PIPELINESTRUCTURE_H

#include <vector>
#include <Configureation.h>
#include "Instruction.h"

class PipelineStructure {
public:
    PipelineStructure();
    ~PipelineStructure();
    struct IF_ID_Register {
        uint32_t pc{};
        Instruction instruction;
        bool valid = true;
    } if_id;
    struct ID_EX_Register {
        uint32_t pc{};
        Instruction instruction;
        uint32_t rs_num{};
        uint32_t rt_num{};
        uint32_t rd_num{};
        uint32_t rs_value{};
        uint32_t rt_value{};
        uint32_t immediate{};
        uint32_t shamt{};
        uint32_t forwarded_rs_value{};
        uint32_t forwarded_rt_value{};
        bool regWrite{};
        bool memRead{};
        bool memWrite{};
        bool memToReg{};
        bool branch{};
        bool jump{};
        uint8_t aluOp{};
        bool aluSrc{};
        bool valid = true;
        uint8_t regDst{};
    } id_ex;
    struct EX_MEM_Register {
        uint32_t pc{};
        Instruction instruction;
        uint32_t rs_num{};
        uint32_t rt_num{};
        uint32_t rd_num{};
        uint32_t alu_result{};
        uint32_t rs_value{};
        uint32_t rt_value{};
        uint32_t shamt{};
        uint32_t immediate{};
        bool regWrite{};
        bool memRead{};
        bool memWrite{};
        bool memToReg{};
        bool branch{};
        bool jump{};
        uint8_t regDst{};
        bool valid = true;
    } ex_mem;
    struct MEM_WB_Register {
        uint32_t pc{};
        Instruction instruction;
        uint32_t rs_num{};
        uint32_t rt_num{};
        uint32_t rd_num{};
        uint32_t alu_result{};
        uint32_t memory_read_data{};
        uint32_t rs_value{};
        uint32_t rt_value{};
        bool regWrite{};
        bool memToReg{};
        bool valid = true;
    } mem_wb;
    bool IF_Done = false;
    bool ID_Done = false;
    bool EX_Done = false;
    bool MEM_Done = false;
    bool WB_Done = false;
    bool stallPipeline = false;
    bool flushPipeline = false;
    void updatePipeline();
    void stall();
    void flush();
private:
    uint32_t pc = 0;
    void updateIF_ID();
    void updateID_EX();
    void updateEX_MEM();
    void updateMEM_WB();
};

#endif //PIPELINESTRUCTURE_H