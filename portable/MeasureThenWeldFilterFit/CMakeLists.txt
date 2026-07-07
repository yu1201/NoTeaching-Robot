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

# Eigen3（header-only，唯一第三方依赖）—— 波纹板拐点管线在 (s, smoothH) 平面上的 2D 线性代数
# （直线 PCA / 方向夹角）。Eigen 头文件已【随附】在 third_party/eigen/ 下，整目录拷走即可编、无需另装/联网。
# 默认用该随附副本；也可显式传 -DEIGEN3_INCLUDE_DIR=<eigen 头目录> 或装好系统 Eigen3 改用系统版。
set(_vendored_eigen "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen")
if(EIGEN3_INCLUDE_DIR)
    target_include_directories(MeasureThenWeldFilterFit PUBLIC ${EIGEN3_INCLUDE_DIR})
elseif(EXISTS "${_vendored_eigen}/Eigen/Dense")
    target_include_directories(MeasureThenWeldFilterFit PUBLIC ${_vendored_eigen})
else()
    find_package(Eigen3 3.3 QUIET NO_MODULE)
    if(TARGET Eigen3::Eigen)
        target_link_libraries(MeasureThenWeldFilterFit PUBLIC Eigen3::Eigen)
    else()
        message(FATAL_ERROR
            "Eigen3 not found: third_party/eigen 副本缺失，系统也未装。请恢复随附副本或传 -DEIGEN3_INCLUDE_DIR=<path>。")
    endif()
endif()

add_executable(MeasureThenWeldFilterFitExample
    examples/example.cpp
)

target_link_libraries(MeasureThenWeldFilterFitExample
    PRIVATE MeasureThenWeldFilterFit
)

# 自包含端到端回归测试（合成波纹 + 双边预平滑），ctest 可运行：cmake --build . && ctest -C Release
enable_testing()
add_executable(MeasureThenWeldFilterFitRegression
    examples/regression_test.cpp
)
target_link_libraries(MeasureThenWeldFilterFitRegression
    PRIVATE MeasureThenWeldFilterFit
)
add_test(NAME regression COMMAND MeasureThenWeldFilterFitRegression)
