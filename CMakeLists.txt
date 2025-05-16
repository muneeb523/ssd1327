# Set minimum CMake version
cmake_minimum_required(VERSION 3.10)

# Set project name
project(ST7735S_Project)

# Define the executable output
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

# Add executable for your example main file (now C++)
add_executable(example_exec Application_code/Test.cpp)

# Add the source files for your project
target_sources(example_exec PRIVATE
    gfx.c                  # Graphics source file
    ssd1327_oled.c            # ST7735S driver source file
)



# Link necessary libraries
target_link_libraries(example_exec PRIVATE 
    gpiod    # GPIO control library
    pthread  # Threading support
    rt       # Real-time clock support (needed for clock_gettime)
    m        # Math functions
)

# Set C and C++ standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)
set(CMAKE_CXX_STANDARD 17)  # Ensure C++17 for Test.cpp
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Ensure C files are compiled with C compiler even in a C++ project
set_source_files_properties(
    gfx.c  ssd1327_oled.c     
    PROPERTIES LANGUAGE C
)

# Print final linking status
message(STATUS "Linking libraries: gpiod, pthread, rt, m")
