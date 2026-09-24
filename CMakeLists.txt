cmake_minimum_required(VERSION 3.16)

project(trainer_aircraft_6dof_nonlinear_model VERSION 1.0.0 LANGUAGES CXX)

option(TRAINER_AIRCRAFT_BUILD_TESTS "Build unit and integration tests" ON)
option(TRAINER_AIRCRAFT_BUILD_EXAMPLES "Build examples" ON)

add_library(trainer_aircraft_6dof
    src/config/T6CConfig.cpp
    src/config/T6CWingSeed.cpp
    src/config/T6CTailSeed.cpp
    src/LoadAccumulator.cpp
    src/TrainerAircraftModel.cpp
    src/RigidBody6DOF.cpp
    src/RK4Integrator.cpp
    src/initialization/GroundStaticTrim.cpp
    src/components/stabilizers/VATC_HorizontalStabilizer.cpp
    src/components/WingTailFlowField.cpp
    src/components/stabilizers/VATC_VerticalStabilizer.cpp
    src/components/fuselage/FuselageComponent.cpp
    src/components/fuselage/FuselageAerodynamics.cpp
    src/components/landing_gear/LandingGearComponent.cpp
    src/components/wings/VATC_MainWing.cpp
    src/components/propeller/PropellerModel.cpp
)

add_library(trainer_aircraft_6dof_stage1 ALIAS trainer_aircraft_6dof)
add_library(trainer_aircraft_6dof_stage2 ALIAS trainer_aircraft_6dof)
add_library(trainer_aircraft_6dof_stage3 ALIAS trainer_aircraft_6dof)
add_library(trainer_aircraft_6dof_stage4 ALIAS trainer_aircraft_6dof)
add_library(trainer_aircraft_6dof_stage5 ALIAS trainer_aircraft_6dof)

target_include_directories(trainer_aircraft_6dof
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(trainer_aircraft_6dof PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(trainer_aircraft_6dof PRIVATE /W4 /permissive-)
else()
    target_compile_options(trainer_aircraft_6dof PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow
    )
endif()

if(TRAINER_AIRCRAFT_BUILD_EXAMPLES)
    add_executable(trainer_aircraft_t6c_wing_seed_demo examples/t6c_wing_seed_demo.cpp)
    target_link_libraries(trainer_aircraft_t6c_wing_seed_demo PRIVATE trainer_aircraft_6dof)
    add_executable(trainer_aircraft_core_example
        examples/minimal_simulation.cpp
    )
    target_link_libraries(trainer_aircraft_core_example PRIVATE trainer_aircraft_6dof)

    add_executable(trainer_aircraft_fuselage_reference_demo
        examples/fuselage_reference_demo.cpp
    )
    target_link_libraries(trainer_aircraft_fuselage_reference_demo PRIVATE trainer_aircraft_6dof)
endif()

if(TRAINER_AIRCRAFT_BUILD_TESTS)
    enable_testing()

    add_executable(trainer_aircraft_core_tests
        tests/test_stage1.cpp
    )
    target_link_libraries(trainer_aircraft_core_tests PRIVATE trainer_aircraft_6dof)

    add_executable(trainer_aircraft_stabilizer_tests
        tests/test_stabilizers.cpp
    )
    target_link_libraries(trainer_aircraft_stabilizer_tests PRIVATE trainer_aircraft_6dof)

    add_executable(trainer_aircraft_propeller_tests
        tests/test_propeller.cpp
    )
    target_link_libraries(trainer_aircraft_propeller_tests PRIVATE trainer_aircraft_6dof)

    add_executable(trainer_aircraft_landing_gear_tests
        tests/test_landing_gear.cpp
    )
    target_link_libraries(trainer_aircraft_landing_gear_tests PRIVATE trainer_aircraft_6dof)

    add_executable(trainer_aircraft_main_wing_tests
        tests/test_main_wing.cpp
    )
    target_link_libraries(trainer_aircraft_main_wing_tests PRIVATE trainer_aircraft_6dof)

    add_executable(trainer_aircraft_fuselage_tests
        tests/test_fuselage.cpp
    )
    target_link_libraries(trainer_aircraft_fuselage_tests PRIVATE trainer_aircraft_6dof)

    add_executable(trainer_aircraft_t6c_config_tests
        tests/test_t6c_config.cpp
    )
    target_link_libraries(trainer_aircraft_t6c_config_tests PRIVATE trainer_aircraft_6dof)

    if(MSVC)
        target_compile_options(trainer_aircraft_core_tests PRIVATE /W4 /permissive-)
        target_compile_options(trainer_aircraft_stabilizer_tests PRIVATE /W4 /permissive-)
        target_compile_options(trainer_aircraft_propeller_tests PRIVATE /W4 /permissive-)
        target_compile_options(trainer_aircraft_landing_gear_tests PRIVATE /W4 /permissive-)
        target_compile_options(trainer_aircraft_main_wing_tests PRIVATE /W4 /permissive-)
        target_compile_options(trainer_aircraft_fuselage_tests PRIVATE /W4 /permissive-)
        target_compile_options(trainer_aircraft_t6c_config_tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(trainer_aircraft_core_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
        target_compile_options(trainer_aircraft_stabilizer_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
        target_compile_options(trainer_aircraft_propeller_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
        target_compile_options(trainer_aircraft_landing_gear_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
        target_compile_options(trainer_aircraft_main_wing_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
        target_compile_options(trainer_aircraft_fuselage_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
        target_compile_options(trainer_aircraft_t6c_config_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
    endif()

    add_test(NAME trainer_aircraft_core_tests COMMAND trainer_aircraft_core_tests)
    add_test(NAME trainer_aircraft_stabilizer_tests COMMAND trainer_aircraft_stabilizer_tests)
    add_test(NAME trainer_aircraft_propeller_tests COMMAND trainer_aircraft_propeller_tests)
    add_test(NAME trainer_aircraft_landing_gear_tests COMMAND trainer_aircraft_landing_gear_tests)
    add_test(NAME trainer_aircraft_main_wing_tests COMMAND trainer_aircraft_main_wing_tests)
    add_test(NAME trainer_aircraft_fuselage_tests COMMAND trainer_aircraft_fuselage_tests)
    add_test(NAME trainer_aircraft_t6c_config_tests COMMAND trainer_aircraft_t6c_config_tests)
endif()
