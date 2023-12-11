project(ATIlib)

# List of source files for the ATIlib library
set(ATIFT_LIB_SOURCES
    src/ATI_FT.cpp
    libs/atidaq/dom.c
    libs/atidaq/expatls.c
    libs/atidaq/ftconfig.c
    libs/atidaq/ftrt.c
    libs/atidaq/node.c
    libs/atidaq/stack.c
    libs/atidaq/xmlparse.c
    libs/atidaq/xmlrole.c
    libs/atidaq/xmltok.c
    )

# Create a static library named ATIlib using the listed source files
add_library(${PROJECT_NAME} STATIC ${ATIFT_LIB_SOURCES})

# Add the necessary include directories for the ATIlib library
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/libs/atidaq)

target_compile_definitions(${PROJECT_NAME} PRIVATE Calibration_Files_Dir="${CMAKE_CURRENT_SOURCE_DIR}/calibration/")


# Check if the build is for Windows
if (WIN32)
    if (NOT DAQmx_DIR)
        message("****")
        set(DAQmx_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs)
    endif()
    find_library(NIDAQmx_LIB 
                NAMES NIDAQmx
                PATHS ${DAQmx_DIR}
                REQUIRED)
endif()

# Check if the build is for Linux
if (LINUX)
    find_library(NIDAQmx_LIB
                NAMES nidaqmx libnidaqmx.so
                PATHS ${PATH_lib}
                REQUIRED)
    # set(libatidaq_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs/LINUX")
    # find_library(ATI_LIB
    #             NAMES libatidaq.so
    #             PATHS ${libatidaq_DIR}
    #             PATH_SUFFIXES so
    #             REQUIRED)
    # target_link_libraries(ATIlib ${ATI_LIB})
endif()


if(NIDAQmx_LIB)
    # Link the NIDAQmx_LIB library to the ATIlib library
    target_link_libraries(${PROJECT_NAME} PRIVATE ${NIDAQmx_LIB})
        message(STATUS ">>> Found NIDAQmx_LIB in:  ${NIDAQmx}")
    else()
        message(FATAL_ERROR ">>> NIDAQmx_LIB not found")
    endif()   


# Find and link Blaze package
set(blaze_DIR /usr/local/include/blaze)
find_package(blaze REQUIRED PATH ${blaze_DIR})
if( blaze_FOUND )
   message(STATUS ">>> Blaze library has been found!")
   target_link_libraries(${PROJECT_NAME} PRIVATE blaze::blaze)
elseif(NOT Blaze_FOUND)
    error(STATUS ">>> Blaze has not been found!")
endif()






