cmake_minimum_required(VERSION 3.16.0)

# PlatformIO passes src/ to CMake as an extra component, idf.py does not.
list(APPEND EXTRA_COMPONENT_DIRS src)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Lesson-6.8_RTOS_miniproject_datalogger)
