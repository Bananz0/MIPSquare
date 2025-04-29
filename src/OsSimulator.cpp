//
// Created by glenm on 4/16/2025.
//

#include "OsSimulator.h"

OSSimulator::OSSimulator() {
    glenCoreUltra = new CPUSimulator();
}

OSSimulator::~OSSimulator() = default;

std::vector<int> OSSimulator::loadProgramInstructions() {
    // //Open the MIPS Program File
    // std::ifstream programRaw("MIPSProgram.txt");
    // if (!programRaw.is_open()) {
    //     std::cerr << "Error: Could not open MIPSProgram.txt\n" << std::endl;
    //     return {};
    // }
    // int myText;
    // while (programRaw >> std::hex >> myText) {
    //     MIPSProgram.push_back(myText);
    //     // if constexpr (PROG_VERBOSE) {
    //     //     printf("Read Value (HEX): 0x%x\n", myText);
    //     // }
    // }
    // if constexpr (PROG_VERBOSE) {
    //     printInstructions();
    // }
    // //Close the file
    // programRaw.close();
    // return MIPSProgram;
    return {};
}

void OSSimulator::printInstructions() {
    // //Used mainly for testng if the read function works and it does
    // std::cout << "===============================" << std::endl;
    // std::cout << "Instructions Read from File" << std::endl;
    // std::cout << "===============================" << std::endl;
    // for (size_t i = 0; i < MIPSProgram.size(); i++) {
    //     std::cout << "Instruction "<< i << ": 0x"<< std::hex <<  MIPSProgram[i] <<std::endl;
    // }
    // std::cout << "===============================" << std::endl;
    // std::cout << "===========    END  ===========" << std::endl;
    // std::cout << "===============================" << std::endl;

    //Moved all of these to the CPU class as that what'll handle the instructions imo
}

void OSSimulator::startSimulation() const {
    glenCoreUltra->startCPU();
}
