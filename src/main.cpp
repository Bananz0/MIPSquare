#include <iostream>

#define DEBUG 1
#define DEBUG_VERBOSITY 0

#include "Register.h"
/*  Thinking of having varying levels of verbosity. According to the MIPS pipeline stages,
I could have debugging outputs on the currently running instructions, possibly clock frequency if I decide to implement
(at the highest verbosity level cause this will probably be annoying)
that,
*/
#define CLOCK_FREQUENCY 1

int main() {
    Register r1(0), r2(1), r3(2), r4(3), r5(4);
    return 0;
}