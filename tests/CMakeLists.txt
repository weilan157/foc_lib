# 单元测试（CTest）：每个测试一个可执行文件
# 条目格式: <目录>:<测试名>，目录相对 tests/
set(FOC_TESTS
    algorithm:test_foc_types
    algorithm:test_config
    algorithm:test_foc_math
    algorithm:test_foc_trig
    algorithm:test_pid
    algorithm:test_cal_abi_align
    algorithm:test_cal_abi_find_index
    algorithm:test_abz_board
    algorithm:test_abs_board
    algorithm:test_enc_abi
    algorithm:test_enc_abs
    service:test_debug_cli
    algorithm:test_current_sense
    algorithm:test_cal_current
    algorithm:test_current_controller
    algorithm:test_current_loop
    runtime:test_command_buffer
    runtime:test_setpoint
    runtime:test_feedback
    runtime:test_config_snapshot
    runtime:test_limiter
    runtime:test_timebase
    runtime:test_event
    control:test_pid_controller
    core:test_pipeline
    safety:test_fault_monitor
    simulation:test_simulation
)

foreach(entry ${FOC_TESTS})
    string(REPLACE ":" ";" parts "${entry}")
    list(GET parts 0 dir)
    list(GET parts 1 t)
    add_executable(${t} ${dir}/${t}.c)
    target_link_libraries(${t} PRIVATE foc_core)
    if(NOT MSVC)
        target_link_libraries(${t} PRIVATE m)
    endif()
    add_test(NAME ${t} COMMAND ${t})
endforeach()
