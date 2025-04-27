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

        PipelineStructure::IF_ID_Register &operator=(const PipelineStructure::IF_ID_Register &other);
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

        PipelineStructure::ID_EX_Register &operator=(const PipelineStructure::ID_EX_Register &other);

        PipelineStructure::ID_EX_Register &operator=(const PipelineStructure::IF_ID_Register &other);
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

        PipelineStructure::EX_MEM_Register &operator=(const PipelineStructure::EX_MEM_Register &other);

        PipelineStructure::EX_MEM_Register &operator=(const PipelineStructure::ID_EX_Register &other);
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

        PipelineStructure::MEM_WB_Register &operator=(const PipelineStructure::MEM_WB_Register &other);

        PipelineStructure::MEM_WB_Register &operator=(const PipelineStructure::EX_MEM_Register &other);
    } mem_wb;
    IF_ID_Register next_if_id;
    ID_EX_Register next_id_ex;
    EX_MEM_Register next_ex_mem;
    MEM_WB_Register next_mem_wb;
    bool IF_Done = false;
    bool ID_Done = false;
    bool EX_Done = false;
    bool MEM_Done = false;
    bool WB_Done = false;


    bool stallPipeline = false;
    bool flushPipeline = false;
private:
    uint32_t pc = 0;
};

#endif //PIPELINESTRUCTURE_H