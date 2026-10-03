cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED IDF_TARGET)
    set(IDF_TARGET "esp32c3")
endif()
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/src")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(kickerboard)

