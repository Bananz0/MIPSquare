# MIPSquare++

A pipelined MIPS processor simulator written in C++ that implements a 5-stage pipeline architecture with hazard detection, data forwarding, and a custom assembly parser.

## Overview

MIPSquare++ (stylized as **glenCoreUltra v1.0 MIPS edition**) is a comprehensive MIPS CPU simulator that executes MIPS assembly programs through a realistic 5-stage pipeline:

1. **Instruction Fetch (IF)**
2. **Instruction Decode (ID)**
3. **Execute (EX)**
4. **Memory Access (MEM)**
5. **Write Back (WB)**

The simulator handles real-world pipeline challenges including data hazards, control hazards, and structural hazards through intelligent forwarding and stalling mechanisms.

## Features

### Core Functionality
- **5-Stage Pipeline**: Fully implemented instruction pipeline with proper stage isolation
- **Hazard Detection & Handling**:
  - Load-use hazard detection with automatic pipeline stalling
  - Data forwarding (EX/MEM and MEM/WB to EX stage)
  - Branch hazard handling with pipeline flushing
- **MIPS Instruction Support**:
  - R-type: `add`, `sub`, `and`, `or`, `slt`, `sll`, `srl`, `sra`, `jr`
  - I-type: `addi`, `lw`, `sw`, `beq`, `bne`
  - J-type: `j`, `jal`
  - Special: `nop`, `stop`

### Advanced Features
- **Assembly Parser**: Converts MIPS assembly directly to machine code
- **Label Resolution**: Automatic label-to-address translation for branches and jumps
- **Register File**: Full 32-register MIPS register file implementation
- **Memory System**: Separate instruction and data memory with word-aligned addressing
- **Debug Output**: Comprehensive pipeline state visualization and instruction tracing

## Architecture

### Pipeline Registers
```
IF/ID → ID/EX → EX/MEM → MEM/WB
```

Each pipeline register contains:
- Instruction data
- Register values
- Control signals
- Validity flags

### Data Forwarding Paths
```
EX/MEM → EX stage (for ALU operations)
MEM/WB → EX stage (for late-arriving data)
```

### Control Signals
- `RegWrite`: Enable register file write
- `MemRead`/`MemWrite`: Memory access control
- `MemToReg`: Select data source for register write
- `Branch`/`Jump`: Control flow signals
- `ALUSrc`: Select ALU input source
- `RegDst`: Select destination register

## Getting Started

### Prerequisites
- CMake 3.30 or higher
- C++23 compatible compiler (GCC/Clang/MSVC)
- Git (for cloning)

### Building

```bash
# Clone the repository
git clone https://github.com/Bananz0/MIPSquare
cd MIPSquare++

# Create build directory
mkdir build && cd build

# Configure and build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .

# Run the executable
./MIPSquared.out
```

### Running Tests

```bash
# From build directory
ctest -C Release
```

## Writing MIPS Programs

### Example Program

```assembly
// Calculate n² using sum of odd numbers
addi $t0, $zero, 0    // Initialize sum
addi $t1, $zero, 1    // First odd number
addi $t2, $zero, 5    // n = 5 (calculate 5²)
addi $t3, $zero, 2000 // Memory address

loop:
    beq $t2, $zero, done
    add $t0, $t0, $t1    // sum += odd
    addi $t1, $t1, 2     // next odd number
    addi $t2, $t2, -1    // counter--
    j loop

done:
    sw $t0, 0($t3)       // Store result
    stop                 // Halt processor
```

### Instruction Format

**R-Type**: `opcode $rd, $rs, $rt`
```assembly
add $s0, $t0, $t1    # $s0 = $t0 + $t1
```

**I-Type**: `opcode $rt, immediate($rs)` or `opcode $rt, $rs, immediate`
```assembly
addi $t0, $zero, 10  # $t0 = 0 + 10
lw $t1, 0($t0)       # $t1 = mem[$t0 + 0]
```

**J-Type**: `opcode label` or `opcode address`
```assembly
j loop               # Jump to label 'loop'
```

### Labels

```assembly
start:               # Label definition
    addi $t0, $zero, 5
    j start          # Jump to label
```

### Comments

```assembly
// Line comment
addi $t0, $zero, 10  // Inline comment
```

## Configuration

Edit `src/Configuration.h` to control debug output:

```cpp
#define DEBUG 1              // Master debug flag
#define FETCH_DEBUG 1        // Fetch stage debug
#define DECODE_DEBUG 1       // Decode stage debug
#define EXECUTE_DEBUG 1      // Execute stage debug
#define MEMORY_DEBUG 1       // Memory stage debug
#define WRITEBACK_DEBUG 1    // Writeback stage debug
#define PIPELINE_DEBUG 1     // Pipeline state debug
#define PARSER_DEBUG 1       // Assembly parser debug
```

## Project Structure

```
MIPSquare++/
├── src/
│   ├── Alu.cpp/h              # ALU operations
│   ├── CpuSimulator.cpp/h     # Main CPU simulator
│   ├── Instruction.cpp/h      # Instruction parsing
│   ├── Memory.cpp/h           # Memory system
│   ├── MipsParser.cpp/h       # Assembly parser
│   ├── PipelineStructure.cpp/h # Pipeline registers
│   ├── ProgramCounter.cpp/h   # PC management
│   ├── Register.cpp/h         # Register implementation
│   ├── RegisterFile.cpp/h     # Register file
│   └── main.cpp               # Entry point
├── tests/                     # Test programs
│   ├── BEQ_J_SW_LW_Test.txt
│   ├── SimpleLoop.txt
│   └── SquareAndStore_Simple.txt
├── CMakeLists.txt
└── MIPSProgram.txt           # Default program
```

## Test Programs

Several test programs are included in the `tests/` directory:

- **SimpleLoop.txt**: Basic loop with branching
- **BEQ_J_SW_LW_Test.txt**: Tests branches, jumps, and memory operations
- **SquareAndStore_Simple.txt**: Calculates n² using odd number summation
- **RAWTest.txt**: Tests read-after-write hazard handling
- **ControlHazardTest.txt**: Tests branch delay slots

## Debugging

### Pipeline State Visualization

The simulator prints detailed pipeline state each cycle:
```
Pipeline State:
  IF/ID: I-type, Opcode: 0x8, rs: 0, rt: 8, immediate: 5
  ID/EX: R-type, Opcode: 0x0, rs: 8, rt: 9, rd: 16, funct: 0x20
  EX/MEM: NOP
  MEM/WB: NOP
```

### Memory Access Tracing

Memory operations are marked with `MAYIHAVEYOURATTENTION`:
```
MEMWRITE ADDRESS: 7d0 with value: 100 MAYIHAVEYOURATTENTION
MEMREAD ADDRESS: 7d0 with value: 100 MAYIHAVEYOURATTENTION
```

## Performance Considerations

- **Pipeline Efficiency**: Achieves near 1 CPI with proper forwarding
- **Hazard Penalties**:
  - Load-use hazard: 1 cycle stall
  - Branch taken: 2 cycles (flush IF/ID)
  - Jump: 1 cycle (flush IF/ID)

## Implementation Details

### Data Forwarding

The simulator implements comprehensive data forwarding:

1. **EX/MEM → EX**: Forwards ALU results directly to dependent instructions
2. **MEM/WB → EX**: Forwards load data or ALU results from WB stage
3. **Priority**: EX/MEM forwarding takes precedence over MEM/WB

### Hazard Detection

```cpp
// Load-use hazard detection
if (ID_EX.memRead && 
    (ID_EX.rt == IF_ID.rs || ID_EX.rt == IF_ID.rt)) {
    // Stall pipeline
    stallPipeline = true;
}
```

### Branch Handling

```cpp
// Branch taken - flush pipeline
if (branchTaken) {
    PC = branch_target;
    IF_ID.valid = false;
    ID_EX.valid = false;
}
```

## Contributing

Contributions are welcome! Areas for improvement:
- Additional MIPS instructions (multiplication, division)
- Cache simulation
- Branch prediction
- Performance metrics
- GUI visualization

## License

This project is available for educational purposes. See individual file headers for specific licensing information.

## Author

Created by **glenm** as an educational MIPS processor simulator.

## Acknowledgments

- MIPS architecture specification
- Computer Organization and Design (Patterson & Hennessy)
- CLion IDE for development support

---

**Note**: This simulator is designed for educational purposes and may not reflect all nuances of real MIPS processors. The instruction set and pipeline behavior are simplified for clarity and learning.
