//
// Created by glenm on 4/16/2025.
//

#ifndef MEMORY_H
#define MEMORY_H

#include <Configureation.h>
#include <stdint.h>
#include <vector>

class Memory {
public:
    Memory();
    ~Memory();

    void setMemory(const std::vector<int>& memDataIn);
    std::vector<int> getMemory();

    uint32_t getMemoryValue(uint32_t address) const;

private:
    std::vector<int> memory;
};


#endif //MEMORY_H
