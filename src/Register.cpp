#include <Register.h>

//#define REGISTER_DEBUG 1
//Moved the Reg debug to the config header

const std::string Register::registerNames[32] = {
  "$zero", "$at", "$v0", "$v1", "$a0", "$a1", "$a2", "$a3",
  "$t0", "$t1", "$t2", "$t3", "$t4", "$t5", "$t6", "$t7",
  "$s0", "$s1", "$s2", "$s3", "$s4", "$s5", "$s6", "$s7",
  "$t8", "$t9", "$k0", "$k1", "$gp", "$sp", "$fp", "$ra"
};

Register::Register(const RegisterNumber registerNumberIn) : registerNumber(registerNumberIn) {
  setRegisterName();
}

void Register::setValue(const int regIn) {
  if (registerNumber == RegisterNumber::zero) {
    if constexpr (REGISTER_DEBUG) {
      std::cerr << "Warning: Attempt to write to $zero register ignored.\n";
    }
    return;
  }
  if constexpr (REGISTER_DEBUG) {
    std::cout << "Register: " << getRegisterName()
              << " Setting value to 0x" << std::hex << regIn << "\n";  }
  registerValue = regIn;
}

int Register::getValue() const {
  return registerValue;
}

std::string Register::getRegisterName() const {
  return registerName;
}

void Register::setRegisterName() {
  int regNum = static_cast<int>(registerNumber);
  assert(regNum >= 0 && regNum < 32);
  if (regNum >= 0 && regNum < 32) {
    registerName = registerNames[regNum];
  } else {
    registerName = "$uninitializedRegister";
    std::cerr << "Unknown register number: " << regNum << "\n";
  }
  if constexpr (REGISTER_DEBUG) {
    std::cout << "Initialized Register: " << registerName << "\n";
  }
}
