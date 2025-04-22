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

    //Pipeline registers
    struct IF_ID_Register {
        uint32_t pc{};
        Instruction readInstruction;
    } if_id;
    struct ID_EX_Register {
        uint32_t instruction;
        uint32_t pc;
    } id_ex;
    struct EX_MEM_Register {
        uint32_t instruction;
        uint32_t pc;
        int rs_value;
        int rt_value;
        uint32_t immediate;
    } ex_mem;
    struct MEM_WB_Register {

    } mem_wb;

private:
    //Current Program Counter
    uint32_t pc = 0;
    //Pipeline control flags
    bool stall = false;
    bool flush = false;
};



#endif //PIPELINESTRUCTURE_H
