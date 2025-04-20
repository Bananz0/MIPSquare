//
// Created by glenm on 4/16/2025.
//

#ifndef MEMORY_H
#define MEMORY_H

#include <Configureation.h>
#include <cstdint>
#include <vector>

class Memory {
public:
    Memory();
    ~Memory();

    void setMemory(const std::vector<uint32_t>& memDataIn);
    std::vector<uint32_t> getMemory();

    void setMemoryValue(uint32_t address, uint32_t value);

    [[nodiscard]] uint32_t getMemoryValue(uint32_t address) const;

private:
    std::vector<uint32_t> memory;
};


#endif //MEMORY_H
