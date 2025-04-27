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
#include <map>
#include "Instruction.h"
#include <Configureation.h>

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

    // Store labels and their corresponding instruction indices
    std::map<std::string, int> labelMap;

    // Store raw instructions for second pass
    std::vector<std::string> rawInstructions;

public:
    MipsParser() {}

    // Get register number from name (e.g., "t1" -> 9)
    int getRegisterNumber(const std::string &regName) {
        // Handle special case with $ prefix
        std::string name = regName;
        if (!name.empty() && name[0] == '$') {
            name = name.substr(1);
        }

        // Check for direct numbers like $1, $2, etc.
        if (!name.empty() && isdigit(name[0])) {
            return std::stoi(name);
        }

        // Handle "zero" special case
        if (name == "zero") {
            return 0;
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

    // Process an instruction and return its machine code
    uint32_t assembleInstruction(const std::string &instruction, int currentInstrIndex) {
        std::istringstream iss(instruction);
        std::string opName;
        iss >> opName;

        if constexpr (PARSER_DEBUG) {
            std::cout << "Assembling instruction: " << instruction << ", index: " << currentInstrIndex << std::endl;
        }

        // Special case for NOP
        if (opName == "nop") {
            return 0x00000000;
        }

        if (opName == "halt") {
            return 0xFFFFFFFF;
        }


        InstructionFormat format = getInstructionFormat(opName);
        uint32_t machineCode = 0;

        if (format.type == InstructionType::R_Instruciton) {
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
                // R-type format: add rs, rt, rd
                std::string reg1Str, reg2Str, reg3Str;

                iss >> reg1Str;
                if (reg1Str.back() == ',') reg1Str.pop_back();

                iss >> reg2Str;
                if (reg2Str.back() == ',') reg2Str.pop_back();

                iss >> reg3Str;

                int rs = getRegisterNumber(reg1Str);
                int rt = getRegisterNumber(reg2Str);
                int rd = getRegisterNumber(reg3Str);

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
                // Branch format: op rs, rt, label/offset
                std::string rsStr, rtStr, labelOrOffset;
                iss >> rsStr;
                if (rsStr.back() == ',') rsStr.pop_back();

                iss >> rtStr;
                if (rtStr.back() == ',') rtStr.pop_back();

                iss >> labelOrOffset;

                int rs = getRegisterNumber(rsStr);
                int rt = getRegisterNumber(rtStr);
                int immediate;

                // Check if it's a direct offset or a label
                if (isdigit(labelOrOffset[0]) ||
                    (labelOrOffset[0] == '-' && labelOrOffset.size() > 1 && isdigit(labelOrOffset[1]))) {
                    // Direct offset
                    immediate = std::stoi(labelOrOffset);
                } else {
                    // It's a label, look up its address
                    auto it = labelMap.find(labelOrOffset);
                    if (it != labelMap.end()) {
                        // Calculate branch offset (target - (current+1))
                        immediate = it->second - (currentInstrIndex + 1);
                    } else {
                        std::cerr << "Undefined label: " << labelOrOffset << std::endl;
                        immediate = 0;
                    }
                }

                machineCode = (format.opcode << 26) | (rs << 21) | (rt << 16) | (immediate & 0xFFFF);
            } else {
                // Generic I-type: addi rt, rs, immediate
                std::string rtStr, rsStr;
                int immVal;

                iss >> rtStr;
                if (rtStr.back() == ',') rtStr.pop_back();

                iss >> rsStr;
                if (rsStr.back() == ',') rsStr.pop_back();

                iss >> immVal;

                int rt = getRegisterNumber(rtStr);
                int rs = getRegisterNumber(rsStr);

                machineCode = (format.opcode << 26) | (rs << 21) | (rt << 16) | (immVal & 0xFFFF);
            }} else if (format.type == InstructionType::J_Instruction) {
                // J-type: op target
                std::string targetStr;
                iss >> targetStr;

                uint32_t target = 0;

                // Check if it's a numerical address or a label
                if (isdigit(targetStr[0]) ||
                    (targetStr[0] == '0' && targetStr.size() > 1 && (targetStr[1] == 'x' || targetStr[1] == 'X'))) {
                    // Direct address
                    if (targetStr.substr(0, 2) == "0x") {
                        target = std::stoul(targetStr, nullptr, 16);
                    } else {
                        target = std::stoul(targetStr);
                    }
                    } else {
                        // It's a label, look up its address
                        auto it = labelMap.find(targetStr);
                        if (it != labelMap.end()) {
                            target = it->second;
                        } else {
                            std::cerr << "Undefined label: " << targetStr << std::endl;
                            target = 0;
                        }
                    }

                // Debug output
                std::cout << "J-Type Instruction: " << opName << " " << targetStr << std::endl;
                std::cout << "Target string: " << targetStr << std::endl;
                std::cout << "Label found, target index = " << target << std::endl;

                // For J instructions, 26-bit target field is the word address
                uint32_t opcodePart = (format.opcode << 26);
                uint32_t targetPart = (target & 0x3FFFFFF);

                std::cout << "Opcode: 0x" << std::hex << opcodePart << std::endl;
                std::cout << "Final target (address): 0x" << std::hex << targetPart << std::endl;

                machineCode = opcodePart | targetPart;

                std::cout << "Final machine code: 0x" << std::hex << machineCode << std::endl;
            }

        return machineCode;
    }

    // Load a MIPS program from a file and return machine code
    std::vector<uint32_t> loadProgramFromFile(const std::string &filename) {
        std::vector<uint32_t> machineCode;
        rawInstructions.clear();
        labelMap.clear();

        std::ifstream programFile(filename);
        if (!programFile.is_open()) {
            std::cerr << "Error: Could not open " << filename << std::endl;
            return {};
        }

        // First pass: collect labels and their instruction indices
        std::string line;
        int instrIndex = 0;
        while (std::getline(programFile, line)) {
            // Trim comments and whitespace
            size_t commentPos = line.find("//");
            if (commentPos != std::string::npos) {
                line = line.substr(0, commentPos);
            }
            line.erase(0, line.find_first_not_of(" \t"));
            if (line.empty()) continue;
            if (line.back() == '\r') line.pop_back();
            line.erase(line.find_last_not_of(" \t") + 1);

            if (line.empty()) continue;

            size_t colonPos = line.find(':');
            if (colonPos != std::string::npos) {
                std::string labelName = line.substr(0, colonPos);
                labelName.erase(0, labelName.find_first_not_of(" \t"));
                labelName.erase(labelName.find_last_not_of(" \t") + 1);

                // Map label to current instruction index
                labelMap[labelName] = instrIndex;

                // Check for instruction after the label
                std::string remaining = line.substr(colonPos + 1);
                remaining.erase(0, remaining.find_first_not_of(" \t"));
                if (!remaining.empty()) {
                    rawInstructions.push_back(remaining);
                    instrIndex++;
                }
            } else if (line.substr(0, 2) == "0x") {
                rawInstructions.push_back(line);
                instrIndex++;
            } else {
                rawInstructions.push_back(line);
                instrIndex++;
            }
        }

        if constexpr (PARSER_DEBUG) {
            std::cout << "First Pass Results:" << std::endl;
            std::cout << "Raw Instructions:" << std::endl;
            for (size_t i = 0; i < rawInstructions.size(); ++i) {
                std::cout << i << ": " << rawInstructions[i] << std::endl;
            }
            std::cout << "Label Map:" << std::endl;
            for (const auto &pair : labelMap) {
                std::cout << pair.first << ": " << pair.second << std::endl;
            }
        }

        // Second pass: resolve labels and assemble instructions
        programFile.clear();
        programFile.seekg(0);
        machineCode.reserve(rawInstructions.size());
        for (int i = 0; i < rawInstructions.size(); ++i) {
            std::string &instr = rawInstructions[i];
            if (instr.substr(0, 2) == "0x") {
                machineCode.push_back(std::stoul(instr, nullptr, 16));
            } else {
                machineCode.push_back(assembleInstruction(instr, i));
            }
        }

        return machineCode;
    }
};

#endif //MIPSPARSER_H