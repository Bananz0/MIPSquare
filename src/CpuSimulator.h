//
// Created by glenm on 4/16/2025.
//

#ifndef CPUSIMULATOR_H
#define CPUSIMULATOR_H

#include <memory>
#include <Configureation.h>
#include <PipelineStructure.h>
#include "Memory.h"
#include "RegisterFile.h"
#include "Alu.h"
#include "Instruction.h"
#include <Multiplexer.h>
#include <ProgramCounter.h>
#include <chrono>
#include <thread>
#include <fstream>

typedef enum {
    FETCH, DECODE, EXECUTE, MEMORY_ACCESS, WRITE_BACK, EXIT
    //I will only access the first five and Exit will only be accessed when there is nothing left on the PC
    //Non standard way of coding i think but idc
} PipelineStages;

class CPUSimulator {
public:
    CPUSimulator();
    ~CPUSimulator();

    static std::vector<uint32_t> loadProgramFromFile();

    //Load Instructions
    void loadProgramInstructions(const std::vector<uint32_t> &memData);
    static void printInstructions(const std::vector<uint32_t> &instructionVector);

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

    void startCPU();

    void virtualClock();

    //Statistics
    int instructionsExecuted = 0;
    int cyclesExecuted = 0;
    bool programLoaded = false;
    bool cpuRunning = false;

private:
    //CPU Clock
    bool clock = false;
    //MiPS Register File
    std::unique_ptr<RegisterFile> regfile;
    //MiPs instruction Memory
    std::unique_ptr<Memory> instructionMemory;
    //MiPs Data Memory
    std::unique_ptr<Memory> dataMemory;
    //PipeLine
    std::unique_ptr<PipelineStructure> pipelineStructure;
    PipelineStages currentStage = FETCH, nextStage = FETCH;
    //ALU
    std::unique_ptr<ALU> alu;
    //PC
    std::unique_ptr<ProgramCounter> programCounter;
    //Multiplexers
    std::unique_ptr<Multiplexer> mux1, mux2, mux3, mux4;

};



#endif //CPUSIMULATOR_H
