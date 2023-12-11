add_executable(${PROJECT_NAME} main.cpp)

# link executable to the udp_library
target_link_libraries(${PROJECT_NAME} ATIlib)