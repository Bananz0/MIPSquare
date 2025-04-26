#include <OsSimulator.h>


#include "MipsParser.h"
#include "Instruction.h"

MipsParser parser;


int main() {
    // const OSSimulator glenOS;
    // glenOS.startSimulation();


    // Parse the assembly file
    std::vector<uint32_t> machineCode = parser.loadProgramFromFile("MIPSProgram.txt");

    // Print out the results
    std::cout << "Assembly to Machine Code Test:\n";
    std::cout << "------------------------------\n";

    // Reopen to display original assembly
    std::ifstream readAssembly("MIPSProgram.txt");
    std::string line;
    int i = 0;

    while (std::getline(readAssembly, line) && i < machineCode.size()) {
        // Skip comment lines and empty lines
        if (line.empty() || (line.find("//") == 0)) {
            continue;
        }

        // Skip lines that only have a label
        if (line.find(':') != std::string::npos &&
            line.find_first_not_of(" \t", line.find(':') + 1) == std::string::npos) {
            continue;
        }

        // Print the assembly line and corresponding machine code
        std::cout << "Assembly: " << line << std::endl;
        std::cout << "Machine code (hex): 0x" << std::hex << machineCode[i] << std::dec << std::endl;
        std::cout << "Machine code (bin): " << std::bitset<32>(machineCode[i]) << std::endl;

        // Create an Instruction object to decode it
        Instruction inst(machineCode[i]);
        std::cout << "Decoded: " << inst.toString() << std::endl;
        std::cout << std::endl;

        i++;
    }
    readAssembly.close();

    // Test with a more complex example that includes a loop
    std::cout << "\nComplex Program with Loop Test:\n";
    std::cout << "------------------------------\n";

    // Create a file with a more complex program
    std::ofstream complexFile("complex_program.txt");
    complexFile << "// Compute sum of numbers from 1 to 10\n";
    complexFile << "addi t0, zero, 0     // Initialize sum\n";
    complexFile << "addi t1, zero, 1     // Counter starts at 1\n";
    complexFile << "addi t2, zero, 10    // Limit\n";
    complexFile << "sum_loop:\n";
    complexFile << "  add t0, t0, t1     // Add current number to sum\n";
    complexFile << "  addi t1, t1, 1     // Increment counter\n";
    complexFile << "  ble t1, t2, sum_loop // Loop if counter <= limit\n";
    complexFile << "  add s0, zero, t0   // Store final result\n";
    complexFile.close();

    // Try to parse it (note: assumes implementation of ble instruction)
    std::vector<uint32_t> complexMachineCode = parser.loadProgramFromFile("complex_program.txt");

    // Print out the machine code
    std::cout << "Generated machine code for complex program:\n";
    for (size_t i = 0; i < complexMachineCode.size(); i++) {
        std::cout << "Instruction " << i << ": 0x" << std::hex << complexMachineCode[i]
                << std::dec << std::endl;
    }

    return 0;
}
