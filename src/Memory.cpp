//
// Created by glenm on 4/16/2025.
//

#include "Memory.h"
#include <cstdio>

Memory::Memory() {
    if constexpr (DEBUG) {
        printf("Memory Bank Initialized\n");
    }
}

Memory::~Memory() {
    if constexpr (DEBUG) {
        printf("Memory Bank Deleted\n");
    }
}
