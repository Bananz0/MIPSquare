//
// Created by glenm on 4/16/2025.
//

#ifndef OSSIMULATOR_H
#define OSSIMULATOR_H

#include <Configureation.h>
#include <CpuSimulator.h>
#include <iostream>
#include <fstream>

class OSSimulator {
public:
    OSSimulator();
    ~OSSimulator();

    std::vector<int> loadProgramInstructions();
    void printInstructions() const;

    void startSimulation() const;

private:
    CPUSimulator *glenCoreUltra;
    std::vector<int> MIPSProgram;
};



#endif //OSSIMULATOR_H
