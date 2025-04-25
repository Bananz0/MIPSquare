//
// Created by glenm on 4/16/2025.
//

#include "PipelineStructure.h"

PipelineStructure::PipelineStructure() {
}

PipelineStructure::~PipelineStructure() {
}

void PipelineStructure::updatePipeline() {
    if constexpr (DEBUG) {
        std::cout << "Updating Pipeline Registers" << std::endl;
    }
    updateMEM_WB();
    updateEX_MEM();
    updateID_EX();
    updateIF_ID();
}

void PipelineStructure::stall() {
    stallPipeline = true;
}

void PipelineStructure::flush() {
    if_id.valid = false;
    id_ex.valid = false;
    ex_mem.valid = false;

    IF_Done = false;
    ID_Done = false;
    EX_Done = false;
}

void PipelineStructure::updateIF_ID() {

    if (stallPipeline) {
        if constexpr (DEBUG) {
            std::cout << "Maintaining IF/ID register due to stall" << std::endl;
        }
        return;
    }
    if constexpr (DEBUG) {
        std::cout << "Updating IF/ID register" << std::endl;
    }
}

void PipelineStructure::updateID_EX() {
    if (stallPipeline) {
        if constexpr (DEBUG) {
            std::cout << "Maintaining ID/EX register due to stall" << std::endl;
        }
        return;
    }
    if (!IF_Done) {
        id_ex.valid = false;
        return;
    }

    id_ex.valid = if_id.valid;
    if constexpr (DEBUG) {
        std::cout << "Updating ID/EX register" << std::endl;
    }
}

void PipelineStructure::updateEX_MEM() {
}

void PipelineStructure::updateMEM_WB() {
}
