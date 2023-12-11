cmake_minimum_required(VERSION 3.22)

# use GCC 12

# Enable Position Independent Code (PIC) for shared libraries
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Add the following line to set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED TRUE)

# Set the project name and version to "Catheter_Robot 1.0"
project (Catheter_Robot VERSION 1.0)

# Set the output directory for generated binary files to the "bin" folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

# Set compiler flags for Debug and Release modes with "-Wall" (enable all warnings)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

# set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fPIC")


# addind the library and main executable directories to the project tree
add_subdirectory(lib_ati)           # Ati Sensor library
add_subdirectory(executable)            # main executable

