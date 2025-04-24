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
        Instruction instruction;
        bool valid = true;
    } if_id;
    struct ID_EX_Register {
        uint32_t pc{};
        Instruction instruction;

        //Register identifiers for hazard detection
        uint32_t rs_num{};
        uint32_t rt_num{};
        uint32_t rd_num{};

        //Register values
        uint32_t rs_value{};
        uint32_t rt_value{};
        uint32_t immediate{};
        uint32_t shamt;

        //Control Signals
        bool regWrite{};
        bool memRead{};
        bool memWrite{};
        bool memToReg{};
        bool branch{};
        bool jump{};
        uint8_t aluOp{};
        bool aluSrc{};
        bool valid = true;
        uint8_t regDst;
    } id_ex;
    struct EX_MEM_Register {
        uint32_t pc{};
        Instruction instruction; // Keep full instruction object

        //Register identifiers (still needed for forwarding)
        uint32_t rs_num{};
        uint32_t rt_num{};
        uint32_t rd_num{}; // destination register

        // Results and values
        uint32_t alu_result{};
        uint32_t rs_value{};
        uint32_t rt_value{}; //Also used as write_data for memory
        uint32_t shamt{};
        uint32_t immediate{};

        // Control signals
        bool regWrite{};
        bool memRead{};
        bool memWrite{};
        bool memToReg{};
        bool branch{};
        bool jump{};
        uint8_t regDst{};

        bool valid = true; //For handling stalls/flushes
    } ex_mem;
    struct MEM_WB_Register {
        uint32_t pc{};
        Instruction instruction;

        // Register identifiers
        uint32_t rd_num{}; // Destination register

        // Results
        uint32_t alu_result{};
        uint32_t memory_read_data{};

        // Control signals
        bool regWrite{};
        bool memToReg{}; // Select between ALU result and memory data

        bool valid = true; // For handling stalls/flushes
    } mem_wb;

    //Pipelien Flags
    bool IF_Done = false;
    bool ID_Done = false;
    bool EX_Done = false;
    bool MEM_Done = false;
    bool WB_Done = false;

private:
    //Current Program Counter
    uint32_t pc = 0;
    //Pipeline control flags
    bool stall = false;
    bool flush = false;
};



#endif //PIPELINESTRUCTURE_H
