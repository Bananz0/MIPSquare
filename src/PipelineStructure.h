//
// Created by glenm on 4/16/2025.
//

#ifndef PIPELINESTRUCTURE_H
#define PIPELINESTRUCTURE_H


#include <Configureation.h>
#include <vector>

#include "Instruction.h"

class PipelineStructure {
public:
    PipelineStructure();
    ~PipelineStructure();

    //Pipeline registers
    struct IF_ID_Register { /* ... */ } if_id;
    struct ID_EX_Register { /* ... */ } id_ex;
    struct EX_MEM_Register { /* ... */ } ex_mem;
    struct MEM_WB_Register { /* ... */ } mem_wb;

private:
    //Current Program Counter
    uint32_t pc = 0;

    //Pipeline control flags
    bool stall = false;
    bool flush = false;
};



#endif //PIPELINESTRUCTURE_H
