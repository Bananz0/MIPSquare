#include <iostream>
#include <string>
#include <Register.h>

#define REGISTER_DEBUG 1


Register::Register(const int registerNumberIn) : registerNumber(registerNumberIn) {
  setRegisterName();
}

void Register::setValue(const int regIn) {
  if constexpr (REGISTER_DEBUG) {
    std::cerr << "Setting value to 0x" << std::hex << regIn << "\n";
  }
  registerValue = regIn;
}

int Register::getValue() const {
  return registerValue;
}

std::string Register::getRegisterName() {
  return registerName;
}

void Register::setRegisterName() {
  switch (registerNumber) {
    case 0:
      registerName = "$zero";
      break;
    case 1:
      registerName = "$at";
      break;
    case 2:
      registerName = "$v0";
      break;
    case 3:
      registerName = "$v1";
      break;
    case 4:
      registerName = "$a0";
      break;
    case 5:
      registerName = "$a1";
      break;
    case 6:
      registerName = "$a2";
      break;
    case 7:
      registerName = "$a3";
      break;
    case 8:
      registerName = "$t0";
      break;
    case 9:
      registerName = "$t1";
      break;
    case 10:
      registerName = "$t2";
      break;
    case 11:
      registerName = "$t3";
      break;
    case 12:
      registerName = "$t4";
      break;
    case 13:
      registerName = "$t5";
      break;
    case 14:
      registerName = "$t6";
      break;
    case 15:
      registerName = "$t7";
      break;
    case 16:
      registerName = "$s0";
      break;
    case 17:
      registerName = "$s1";
      break;
    case 18:
      registerName = "$s2";
      break;
    case 19:
      registerName = "$s3";
      break;
    case 20:
      registerName = "$s4";
      break;
    case 21:
      registerName = "$s5";
      break;
    case 22:
      registerName = "$s6";
      break;
    case 23:
      registerName = "$s7";
      break;
    case 24:
      registerName = "$t8";
      break;
    case 25:
      registerName = "$t9";
      break;
    case 26:
      registerName = "$k0";
      break;
    case 27:
      registerName = "$k1";
      break;
    case 28:
      registerName = "$gp";
      break;
    case 29:
      registerName = "$sp";
      break;
    case 30:
      registerName = "$fp";
      break;
    case 31:
      registerName = "$ra";
      break;
    default:
      registerName = "$uninitializedRegister";
      std::cerr << "Unknown register number: " << registerNumber << "\n";
  }
  if constexpr (REGISTER_DEBUG) {
    std::cerr << "Initialized Register: " << registerName << "\n";
  }
}
