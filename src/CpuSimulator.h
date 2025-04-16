//
// Created by glenm on 4/16/2025.
//

#ifndef CPUSIMULATOR_H
#define CPUSIMULATOR_H

#include <PipelineStructure.h>
#include "Memory.h"
#include "RegisterFile.h"
#include <Configureation.h>

class CPUSimulator {
public:
    CPUSimulator();
    ~CPUSimulator();

private:
    //MiPS Register File
    RegisterFile *regfile;
    //MiPs instruction Memory
    Memory *instructionMemory;
    //MiPs Data Memory
    Memory *dataMemory;
    //PipeLine


};



#endif //CPUSIMULATOR_H
