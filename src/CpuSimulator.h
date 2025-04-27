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
} PipelineStages; //Moved away from this
class CPUSimulator {
public:
    CPUSimulator();
    ~CPUSimulator();

    //Load Instructions
    void loadProgramInstructions(const std::vector<uint32_t> &memData);
    static void printInstructions(const std::vector<uint32_t> &instructionVector);
    [[nodiscard]] bool detectLoadUseHazard() const;
    static std::vector<uint32_t> loadProgramFromFile();

    //Pipeline Stages
    void fetch() const;
    void handleBranchHazard(bool taken, uint32_t target_pc) const;
    void decode() const;
    void execute();
    void memoryAccess() const;
    void writeBack() const;

    // //Hazard detection and handling
    // bool detectDataHazard();
    // bool detectControlHazard();
    // void handleHazard();

    //Data Forwarding
    void dataForwarder(uint32_t &input1, uint32_t &input2);
    void startCPU();
    void virtualClock();
    void printPipelineState() const;
    static std::string getRegisterName(uint8_t regNum);

    void setControlSignals(const Instruction &instr) const;

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

    //Pipeline Register values for data
    uint32_t instructionFetch{};
    uint32_t readData1{};
    uint32_t readData2{};
    uint32_t aluResult{};
    uint32_t memoryReadData{};

    //Statistics
    int instructionsExecuted = 0;
    int cyclesExecuted = 0;
    bool programLoaded = false;
    bool cpuRunning = false;

    // Control Signals for Each Stage (Example, add more as needed)
    bool regWrite{}; // WB Stage
    bool memRead{}; // MEM Stage
    bool memWrite{}; // MEM Stage
    bool aluOp{}; // EX Stage
};



#endif //CPUSIMULATOR_H
