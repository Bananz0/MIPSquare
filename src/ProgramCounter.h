//
// Created by glenm on 4/20/2025.
//

#ifndef PROGRAMCOUNTER_H
#define PROGRAMCOUNTER_H
#include <cstdint>
#include <iostream>
#include <Configureation.h>



class ProgramCounter {
public:
    ProgramCounter();
    ~ProgramCounter();

    uint32_t getPC() const;
    void setPC(uint32_t newPC);

    void incrementPC();
    void decrementPC();

    void addPC(uint32_t amount);
    void subtractPC(uint32_t amount);
    void jumpTo(uint32_t newPC);

    void updatePC();
private:
    uint32_t currentPc, nextPc, prevPc;
};

#endif //PROGRAMCOUNTER_H
