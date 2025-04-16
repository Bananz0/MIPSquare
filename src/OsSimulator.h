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

    void loadProgramInstructions();
    void printInstructions();

private:
    CPUSimulator *glenCoreUltra;
    std::vector<std::string> MIPSProgram;
};



#endif //OSSIMULATOR_H
