//
// Created by glenm on 4/16/2025.
//

#include "OsSimulator.h"

OSSimulator::OSSimulator() {
    glenCoreUltra = new CPUSimulator();
}

OSSimulator::~OSSimulator() = default;

void OSSimulator::loadProgramInstructions() {
    //Open the MIPS Program File
    std::ifstream programRaw("MIPSProgram.txt");

    std::string myText;
    while (getline (programRaw, myText)) {
        MIPSProgram.push_back(myText);
    }

    //Close the file
    programRaw.close();

}

void OSSimulator::printInstructions() {

}
