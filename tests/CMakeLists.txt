# 单元测试（CTest）：每个测试一个可执行文件
set(FOC_TESTS
    test_foc_types
    test_config
    test_foc_math
    test_foc_trig
    test_pid
    test_command_buffer
    test_setpoint
    test_feedback
    test_config_snapshot
    test_limiter
    test_timebase
    test_event
    test_fault_monitor
    test_pid_controller
    test_pipeline
    test_simulation
)

foreach(t ${FOC_TESTS})
    add_executable(${t} algorithm/${t}.c)
    target_link_libraries(${t} PRIVATE foc_core)
    if(NOT MSVC)
        target_link_libraries(${t} PRIVATE m)
    endif()
    add_test(NAME ${t} COMMAND ${t})
endforeach()
