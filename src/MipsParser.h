//
// Created by glenm on 4/24/2025.
//

#ifndef MIPSPARSER_H
#define MIPSPARSER_H



#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include "Instruction.h"

class MipsParser {
private:
    // Register mapping
    static const int MAX_REGISTERS = 32;
    const char *registerNames[MAX_REGISTERS] = {
        "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3",
        "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
        "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
        "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"
    };

    // Instruction mapping
    struct InstructionFormat {
        InstructionType type;
        uint8_t opcode;
        uint8_t funct;
    };

    // Simple array-based approach for instruction formats
    static const int MAX_INSTRUCTIONS = 20;

    struct {
        const char *name;
        InstructionFormat format;
    } instructionFormats[MAX_INSTRUCTIONS] = {
        {"add", {InstructionType::R_Instruciton, 0x00, InstructionSet::ADD}},
        {"sub", {InstructionType::R_Instruciton, 0x00, InstructionSet::SUB}},
        {"and", {InstructionType::R_Instruciton, 0x00, InstructionSet::AND}},
        {"or", {InstructionType::R_Instruciton, 0x00, InstructionSet::OR}},
        {"slt", {InstructionType::R_Instruciton, 0x00, InstructionSet::SLT}},
        {"sll", {InstructionType::R_Instruciton, 0x00, InstructionSet::SLL}},
        {"srl", {InstructionType::R_Instruciton, 0x00, InstructionSet::SRL}},
        {"sra", {InstructionType::R_Instruciton, 0x00, InstructionSet::SRA}},
        {"jr", {InstructionType::R_Instruciton, 0x00, InstructionSet::JR}},
        {"addi", {InstructionType::I_Instruction, InstructionSet::ADDI, 0}},
        {"slladdi", {InstructionType::I_Instruction, InstructionSet::SLLADDI, 0}},
        {"lw", {InstructionType::I_Instruction, InstructionSet::LW, 0}},
        {"sw", {InstructionType::I_Instruction, InstructionSet::SW, 0}},
        {"beq", {InstructionType::I_Instruction, InstructionSet::BEQ, 0}},
        {"bne", {InstructionType::I_Instruction, InstructionSet::BNE, 0}},
        {"j", {InstructionType::J_Instruction, InstructionSet::J, 0}},
        {"jal", {InstructionType::J_Instruction, InstructionSet::JAL, 0}}
    };

    static const int MAX_LABELS = 100;

    struct Label {
        std::string name;
        int address;
    };

    Label labels[MAX_LABELS];
    int labelCount = 0;

    std::vector<std::string> rawInstructions;

public:
    MipsParser() : labelCount(0) {
    }

    // Get register number from name (e.g., "t1" -> 9)
    int getRegisterNumber(const std::string &regName) {
        // Handle special case with $ prefix
        std::string name = regName;
        if (name[0] == '$') {
            name = name.substr(1);
        }

        // Check for direct numbers like $1, $2, etc.
        if (isdigit(name[0])) {
            return std::stoi(name);
        }

        // Look up in register names
        for (int i = 0; i < MAX_REGISTERS; i++) {
            if (name == registerNames[i]) {
                return i;
            }
        }

        std::cerr << "Unknown register: " << regName << std::endl;
        return -1;
    }

    // Get instruction format from name
    InstructionFormat getInstructionFormat(const std::string &name) {
        for (int i = 0; i < MAX_INSTRUCTIONS; i++) {
            if (name == instructionFormats[i].name) {
                return instructionFormats[i].format;
            }
        }
        std::cerr << "Unknown instruction: " << name << std::endl;
        return {InstructionType::Uninitialized, 0, 0};
    }

    // Parse and add a label
    void addLabel(const std::string &name, int address) {
        if (labelCount < MAX_LABELS) {
            // Remove trailing colon if present
            std::string labelName = name;
            if (labelName.back() == ':') {
                labelName.pop_back();
            }
            labels[labelCount].name = labelName;
            labels[labelCount].address = address;
            labelCount++;
        } else {
            std::cerr << "Too many labels, max is " << MAX_LABELS << std::endl;
        }
    }

    // Find label address
    int findLabelAddress(const std::string &name) {
        for (int i = 0; i < labelCount; i++) {
            if (labels[i].name == name) {
                return labels[i].address;
            }
        }
        return -1; // Label not found
    }

// Assemble a single instruction to binary
    uint32_t assembleInstruction(const std::string &instruction) {
        std::istringstream iss(instruction);
        std::string opName;
        iss >> opName;

        // Check if this is a label (ends with :)
        if (opName.back() == ':') {
            // Get the next token which should be the actual instruction
            iss >> opName;
        }

        InstructionFormat format = getInstructionFormat(opName);

        uint32_t machineCode = 0;

        if (format.type == InstructionType::R_Instruciton) {
            // R-type instructions
            if (opName == "sll" || opName == "srl" || opName == "sra") {
                // Shift format: op rd, rt, shamt
                std::string rdStr, rtStr;
                int shamtVal;

                iss >> rdStr;
                if (rdStr.back() == ',') rdStr.pop_back();

                iss >> rtStr;
                if (rtStr.back() == ',') rtStr.pop_back();

                iss >> shamtVal;

                int rd = getRegisterNumber(rdStr);
                int rt = getRegisterNumber(rtStr);

                machineCode = (format.opcode << 26) | (0 << 21) | (rt << 16) |
                              (rd << 11) | (shamtVal << 6) | format.funct;
            } else if (opName == "jr") {
                // Jump register: jr rs
                std::string rsStr;
                iss >> rsStr;
                int rs = getRegisterNumber(rsStr);

                machineCode = (format.opcode << 26) | (rs << 21) |
                              (0 << 16) | (0 << 11) | (0 << 6) | format.funct;
            } else {
                // Your custom R-type format is: add rs rt rd
                // Where:
                // - First register is rs (source 1)
                // - Second register is rt (source 2)
                // - Third register is rd (destination)
                std::string rsStr, rtStr, rdStr;

                // Read the three registers in your custom order
                iss >> rsStr; // First register (rs)
                if (rsStr.back() == ',') rsStr.pop_back();

                iss >> rtStr; // Second register (rt)
                if (rtStr.back() == ',') rtStr.pop_back();

                iss >> rdStr; // Third register (rd)

                // Get register numbers
                int rs = getRegisterNumber(rsStr);
                int rt = getRegisterNumber(rtStr);
                int rd = getRegisterNumber(rdStr);

                // Create the machine code with the correct field positions
                // Format for R-type: opcode(6) | rs(5) | rt(5) | rd(5) | shamt(5) | funct(6)
                machineCode = (format.opcode << 26) | (rs << 21) | (rt << 16) |
                              (rd << 11) | (0 << 6) | format.funct;
            }
        } else if (format.type == InstructionType::I_Instruction) {
            if (opName == "lw" || opName == "sw") {
                // Memory format: op rt, imm(rs)
                std::string rtStr, offsetBase;
                iss >> rtStr;
                if (rtStr.back() == ',') rtStr.pop_back();

                iss >> offsetBase;

                // Parse "imm(rs)" format
                size_t openParen = offsetBase.find('(');
                size_t closeParen = offsetBase.find(')');

                if (openParen != std::string::npos && closeParen != std::string::npos) {
                    int immediate = std::stoi(offsetBase.substr(0, openParen));
                    std::string rsStr = offsetBase.substr(openParen + 1, closeParen - openParen - 1);

                    int rt = getRegisterNumber(rtStr);
                    int rs = getRegisterNumber(rsStr);

                    machineCode = (format.opcode << 26) | (rs << 21) | (rt << 16) | (immediate & 0xFFFF);
                }
            } else if (opName == "beq" || opName == "bne") {
                // Branch format: op rs, rt, label
                std::string rsStr, rtStr, labelStr;
                iss >> rsStr;
                if (rsStr.back() == ',') rsStr.pop_back();

                iss >> rtStr;
                if (rtStr.back() == ',') rtStr.pop_back();

                iss >> labelStr;

                int rs = getRegisterNumber(rsStr);
                int rt = getRegisterNumber(rtStr);

                // Find label address (for second pass)
                int labelAddr = findLabelAddress(labelStr);
                int immediate = 0;

                if (labelAddr != -1) {
                    // Calculate offset (address difference - 1 for branch delay slot)
                    // We need to account for the current instruction position
                    int currentAddr = rawInstructions.size() - 1;
                    immediate = labelAddr - currentAddr - 1;
                }

                machineCode = (format.opcode << 26) | (rs << 21) | (rt << 16) | (immediate & 0xFFFF);
            } else {
                // Your custom I-type format is: addi rt rs imm
                // Where:
                // - First register is rt (destination)
                // - Second register is rs (source)
                // - Third value is immediate
                std::string rtStr, rsStr;
                int immediate;

                iss >> rtStr; // First argument (rt)
                if (rtStr.back() == ',') rtStr.pop_back();

                iss >> rsStr; // Second argument (rs)
                if (rsStr.back() == ',') rsStr.pop_back();

                iss >> immediate; // Third argument (immediate value)

                int rt = getRegisterNumber(rtStr);
                int rs = getRegisterNumber(rsStr);

                // Create the machine code with the correct field positions
                // Format for I-type: opcode(6) | rs(5) | rt(5) | immediate(16)
                machineCode = (format.opcode << 26) | (rs << 21) | (rt << 16) | (immediate & 0xFFFF);
            }
        } else if (format.type == InstructionType::J_Instruction) {
            // J-type: op target
            std::string targetStr;
            iss >> targetStr;

            uint32_t target = 0;

            // Check if it's a label or direct address
            if (isdigit(targetStr[0]) || targetStr[0] == '0' && (targetStr[1] == 'x' || targetStr[1] == 'X')) {
                // Direct address
                if (targetStr.substr(0, 2) == "0x") {
                    target = std::stoul(targetStr, nullptr, 16);
                } else {
                    target = std::stoul(targetStr);
                }
            } else {
                // Label - look up in label table
            int labelAddr = findLabelAddress(targetStr);
            if (labelAddr != -1) {
                target = labelAddr;
            }
        }

        // The target is only 26 bits and gets shifted left by 2
        target = target >> 2;
        machineCode = (format.opcode << 26) | (target & 0x3FFFFFF);
    }

    return machineCode;
}

    // Load and assemble program from file
    std::vector<uint32_t> loadProgramFromFile(const std::string &filename) {
        std::vector<uint32_t> MIPSProgram;
        rawInstructions.clear();
        labelCount = 0;

        // Open the file
        std::ifstream programFile(filename);
        if (!programFile.is_open()) {
            std::cerr << "Error: Could not open " << filename << std::endl;
            return {};
        }

        // First pass: collect all instructions and identify labels
        std::string line;
        while (std::getline(programFile, line)) {
            // Remove comments
            size_t commentPos = line.find("//");
            if (commentPos != std::string::npos) {
                line = line.substr(0, commentPos);
            }

            // Trim whitespace
            line.erase(0, line.find_first_not_of(" \t"));
            line.erase(line.find_last_not_of(" \t") + 1);

            // Skip empty lines
            if (line.empty()) {
                continue;
            }

            // Check if this is a label declaration
            if (line.find(':') != std::string::npos) {
                std::istringstream iss(line);
                std::string label;
                std::getline(iss, label, ':');

                // Add label with current instruction index
                addLabel(label, rawInstructions.size());

                // Check if there's an instruction on the same line after the label
                std::string restOfLine;
                std::getline(iss, restOfLine);
                restOfLine.erase(0, restOfLine.find_first_not_of(" \t"));

                if (!restOfLine.empty()) {
                    // Add remaining instruction
                    rawInstructions.push_back(restOfLine);
                }
            } else if (line.substr(0, 2) == "0x") {
                // This is already a hex instruction, add it directly
                uint32_t hexValue = std::stoul(line, nullptr, 16);
                MIPSProgram.push_back(hexValue);
            } else {
                // Regular instruction
                rawInstructions.push_back(line);
            }
        }

        // Second pass: assemble instructions now that we have all labels
        for (const std::string &instr: rawInstructions) {
            uint32_t machineCode = assembleInstruction(instr);
            MIPSProgram.push_back(machineCode);
        }

        // Close the file
        programFile.close();

        return MIPSProgram;
    }
};


#endif //MIPSPARSER_H
