# foc_lib —— V0.1.6 Architecture Baseline（Freeze Approved）
# PC 构建：纯算法核心 + 单元测试（CTest）
cmake_minimum_required(VERSION 3.16)
project(foc_lib C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Werror=implicit-function-declaration)
endif()

# 纯算法核心静态库（无 HAL / RTOS / 全局变量，PC 可测）
# V0.1.6 实现阶段①：runtime/control/safety/core + simulation（PC 闭环）
add_library(foc_core STATIC
    src/foc/foc_types.c
    src/foc/config.c
    src/foc/foc_math.c
    src/foc/foc_trig.c
    src/foc/pid.c
    src/runtime/command_buffer.c
    src/runtime/setpoint.c
    src/runtime/feedback.c
    src/runtime/config_snapshot.c
    src/runtime/timebase.c
    src/runtime/limiter.c
    src/runtime/stats.c
    src/runtime/event.c
    src/runtime/telemetry.c
    src/control/pid_controller.c
    src/safety/fault.c
    src/safety/fault_monitor.c
    src/safety/safety.c
    src/core/motor_runtime.c
    tests/simulation/motor_model.c
    tests/simulation/hw_adapter_sim.c
)
target_include_directories(foc_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
)

enable_testing()
add_subdirectory(tests)
