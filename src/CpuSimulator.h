//
// Created by glenm on 4/16/2025.
//

#ifndef CPUSIMULATOR_H
#define CPUSIMULATOR_H

#include <Configureation.h>

#include <PipelineStructure.h>
#include "Memory.h"
#include "RegisterFile.h"
#include "Alu.h"

class CPUSimulator {
public:
    CPUSimulator();
    ~CPUSimulator();

    //Pipeline Stages
    void fetch();
    void decode();
    void execute();
    void memoryAccess();
    void writeBack();

    //Hazard detection and handling
    bool detectDataHazard();
    bool detectControlHazard();
    void handleHazard();

    //Data Forwarding
    void dataForwarder();

    //Parse the program file into instructions
    std::vector<Instruction> parseProgram(const std::string& content);

    //Statistics
    int instructionsExecuted = 0;
    int cyclesExecuted = 0;
    bool programLoaded = false;

private:
    //MiPS Register File
    RegisterFile *regfile;
    //MiPs instruction Memory
    Memory *instructionMemory;
    //MiPs Data Memory
    Memory *dataMemory;
    //PipeLine
    PipelineStructure *pipelineStructure;
    //ALU
    ALU *alu;
};



#endif //CPUSIMULATOR_H
