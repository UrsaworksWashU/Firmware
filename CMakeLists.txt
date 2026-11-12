cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Firmware)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Recursively collect user code from the numbered layer folders (01_bsp, 02_module, 03_application, ...)
# CONFIGURE_DEPENDS makes CMake re-run the glob on each build, so new files/folders are picked up automatically
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_SOURCE_DIR}/[0-9][0-9]_*/*.c
        ${CMAKE_SOURCE_DIR}/[0-9][0-9]_*/*.s)
file(GLOB_RECURSE USER_HEADERS CONFIGURE_DEPENDS
        ${CMAKE_SOURCE_DIR}/[0-9][0-9]_*/*.h)

# Every folder that contains a header becomes an include path
set(USER_INCLUDE_DIRS "")
foreach(header ${USER_HEADERS})
    get_filename_component(dir ${header} DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS ${dir})
endforeach()
list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        ${USER_INCLUDE_DIRS}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
