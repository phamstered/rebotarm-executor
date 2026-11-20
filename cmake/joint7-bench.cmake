option(MOTOR_BENCH_JOINT7_POSVEL "Enable unqualified joint7-only POS_VEL bench image" OFF)
option(MOTOR_BENCH_OVERSPEED_STOP "Stop bench motion above 0.1 rad/s feedback" ON)
option(MOTOR_BENCH_EFFORT_STOP "Stop bench motion above 0.30 Nm reported effort" ON)
if(NOT MOTOR_BENCH_JOINT7_POSVEL AND (NOT MOTOR_BENCH_OVERSPEED_STOP OR NOT MOTOR_BENCH_EFFORT_STOP))
    message(FATAL_ERROR "Disabling speed/effort stops requires the explicit joint7 bench image")
endif()
foreach(parameter POSITION_MIN POSITION_MAX VELOCITY_MAX)
    set(MOTOR_BENCH_${parameter} "" CACHE STRING "Explicit joint7 bench ${parameter} (rad or rad/s)")
endforeach()
if(MOTOR_BENCH_JOINT7_POSVEL)
    foreach(parameter POSITION_MIN POSITION_MAX VELOCITY_MAX)
        if(NOT "${MOTOR_BENCH_${parameter}}" MATCHES "^-?[0-9]+\\.[0-9]+$")
            message(FATAL_ERROR "MOTOR_BENCH_${parameter} requires an explicit finite decimal, e.g. 0.05")
        endif()
    endforeach()
    if(NOT MOTOR_BENCH_POSITION_MIN LESS MOTOR_BENCH_POSITION_MAX OR
       MOTOR_BENCH_POSITION_MIN LESS -12.5 OR MOTOR_BENCH_POSITION_MAX GREATER 12.5 OR
       NOT MOTOR_BENCH_VELOCITY_MAX GREATER 0 OR MOTOR_BENCH_VELOCITY_MAX GREATER 0.05)
        message(FATAL_ERROR "Invalid bench bounds: min < max inside feedback range, 0 < speed <= 0.05 rad/s required")
    endif()
    # This target is deliberately distinct from the production Debug/Release images.
    if(CMAKE_BINARY_DIR STREQUAL "${CMAKE_SOURCE_DIR}/build/Debug" OR
       CMAKE_BINARY_DIR STREQUAL "${CMAKE_SOURCE_DIR}/build/Release")
        message(FATAL_ERROR "Use a dedicated bench build directory (-B build/BenchJoint7Debug)")
    endif()
    if(MOTOR_BENCH_OVERSPEED_STOP)
        set(bench_overspeed_stop 1)
    else()
        set(bench_overspeed_stop 0)
        message(WARNING "SUPERVISED BENCH: overspeed stop OFF; speed feedback still recorded, other guards remain")
    endif()
    if(MOTOR_BENCH_EFFORT_STOP)
        set(bench_effort_stop 1)
    else()
        set(bench_effort_stop 0)
        message(WARNING "SUPERVISED BENCH: effort stop OFF; effort feedback still recorded, drive-native protection unchanged")
    endif()
    add_compile_definitions(MOTOR_BENCH_JOINT7_POSVEL=1
        MOTOR_BENCH_EFFORT_STOP=${bench_effort_stop}
        MOTOR_BENCH_OVERSPEED_STOP=${bench_overspeed_stop}
        MOTOR_BENCH_POSITION_MIN=${MOTOR_BENCH_POSITION_MIN}f
        MOTOR_BENCH_POSITION_MAX=${MOTOR_BENCH_POSITION_MAX}f
        MOTOR_BENCH_VELOCITY_MAX=${MOTOR_BENCH_VELOCITY_MAX}f)
    message(WARNING "UNQUALIFIED joint7-only bench image: ${MOTOR_BENCH_POSITION_MIN}..${MOTOR_BENCH_POSITION_MAX} rad, ${MOTOR_BENCH_VELOCITY_MAX} rad/s")
endif()
