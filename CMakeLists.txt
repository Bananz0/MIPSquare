cmake_minimum_required(VERSION 3.30)
project(MIPSquare__ C CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_COMPILER gcc)

# Important project paths
set(BASE_PATH "${CMAKE_SOURCE_DIR}")
set(SRC_PATH "${BASE_PATH}/src")

# Collect source files recursively
file(GLOB_RECURSE SRC_FILES
        "${SRC_PATH}/*.cpp"
        "${SRC_PATH}/*.cc"
        "${SRC_PATH}/*.c"
        "${SRC_PATH}/*.cxx"
)

# Set compiler flags
set(CWARN "-Wall -Wextra")
set(COPT "-O3")  # Optimization level 2 for better performance
set(CDEBUG "-g")   # Debugging symbols
set(CFLAGS "${COPT} ${CWARN} ${CDEBUG}")
set(CXXFLAGS "${COPT} ${CWARN} ${CDEBUG}")

set(CMAKE_C_FLAGS "${CFLAGS}")
set(CMAKE_CXX_FLAGS "${CXXFLAGS}")

# Include directories
include_directories(${SRC_PATH})

# Create executable
add_executable(${PROJECT_NAME} ${SRC_FILES})

# Set output name
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${PROJECT_NAME}.out")

# Optional cleanup targets
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${PROJECT_NAME}.out")