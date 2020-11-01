cmake_minimum_required(VERSION 3.17)
project(moteus_protocol_python)

set(CMAKE_CXX_STANDARD 20)

find_package(pybind11 REQUIRED)

pybind11_add_module(moteus_protocol MODULE "moteus_protocol_python.cpp")

set_target_properties(moteus_protocol
        PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}"
        )
