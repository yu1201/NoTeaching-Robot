cmake_minimum_required(VERSION 3.12)
project(MeasureThenWeldFilterFit LANGUAGES CXX)

add_library(MeasureThenWeldFilterFit
    src/MeasureThenWeldFilterFit.cpp
)

target_include_directories(MeasureThenWeldFilterFit
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(MeasureThenWeldFilterFit PUBLIC cxx_std_17)

add_executable(MeasureThenWeldFilterFitExample
    examples/example.cpp
)

target_link_libraries(MeasureThenWeldFilterFitExample
    PRIVATE MeasureThenWeldFilterFit
)
