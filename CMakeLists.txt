cmake_minimum_required(VERSION 3.16)

project(trainer_aircraft_6dof_oop_stage5_rpm_ramp_30s VERSION 5.3.0 LANGUAGES CXX)

option(TRAINER_AIRCRAFT_BUILDTESTS "Build the TrainerAircraft unit and integration tests" ON)
option(TRAINER_AIRCRAFT_BUILDEXAMPLES "Build the TrainerAircraft examples" ON)

add_library(trainer_aircraft_6dof
    src/config/ProvisionalTrainerAircraftConfig.cpp
    src/LoadAccumulator.cpp
    src/TrainerAircraftModel.cpp
    src/RigidBody6DOF.cpp
    src/RK4Integrator.cpp
    src/initialization/GroundStaticTrim.cpp
    src/components/stabilizers/VATC_HorizontalStabilizer.cpp
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

if(TRAINER_AIRCRAFT_BUILDEXAMPLES)
    add_executable(trainer_aircraft_core_example
        examples/minimal_simulation.cpp
    )
    target_link_libraries(trainer_aircraft_core_example PRIVATE trainer_aircraft_6dof)

    add_executable(trainer_aircraft_stabilizer_example
        examples/stabilizer_integration.cpp
    )
    target_link_libraries(trainer_aircraft_stabilizer_example PRIVATE trainer_aircraft_6dof)

    add_executable(trainer_aircraft_stage3_example
        examples/stage3_integration.cpp
    )
    target_link_libraries(trainer_aircraft_stage3_example PRIVATE trainer_aircraft_6dof)

    add_executable(trainer_aircraft_stage4_takeoff_roll
        examples/stage4_takeoff_roll.cpp
    )
    target_link_libraries(trainer_aircraft_stage4_takeoff_roll PRIVATE trainer_aircraft_6dof)

    add_executable(trainer_aircraft_stage5_full_aircraft_takeoff_roll
        examples/stage5_full_aircraft_takeoff_roll.cpp
    )
    target_link_libraries(trainer_aircraft_stage5_full_aircraft_takeoff_roll PRIVATE trainer_aircraft_6dof)

    add_executable(trainer_aircraft_fuselage_reference_demo
        examples/fuselage_reference_demo.cpp
    )
    target_link_libraries(trainer_aircraft_fuselage_reference_demo PRIVATE trainer_aircraft_6dof)
endif()

if(TRAINER_AIRCRAFT_BUILDTESTS)
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

    add_executable(trainer_aircraft_full_aircraft_tests
        tests/test_full_aircraft.cpp
    )
    target_link_libraries(trainer_aircraft_full_aircraft_tests PRIVATE trainer_aircraft_6dof)

    add_executable(trainer_aircraft_ground_static_trim_tests
        tests/test_ground_static_trim.cpp
    )
    target_link_libraries(trainer_aircraft_ground_static_trim_tests PRIVATE trainer_aircraft_6dof)

    if(MSVC)
        target_compile_options(trainer_aircraft_core_tests PRIVATE /W4 /permissive-)
        target_compile_options(trainer_aircraft_stabilizer_tests PRIVATE /W4 /permissive-)
        target_compile_options(trainer_aircraft_propeller_tests PRIVATE /W4 /permissive-)
        target_compile_options(trainer_aircraft_landing_gear_tests PRIVATE /W4 /permissive-)
        target_compile_options(trainer_aircraft_main_wing_tests PRIVATE /W4 /permissive-)
        target_compile_options(trainer_aircraft_fuselage_tests PRIVATE /W4 /permissive-)
        target_compile_options(trainer_aircraft_full_aircraft_tests PRIVATE /W4 /permissive-)
        target_compile_options(trainer_aircraft_ground_static_trim_tests PRIVATE /W4 /permissive-)
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
        target_compile_options(trainer_aircraft_full_aircraft_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
        target_compile_options(trainer_aircraft_ground_static_trim_tests PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
        )
    endif()

    add_test(NAME trainer_aircraft_core_tests COMMAND trainer_aircraft_core_tests)
    add_test(NAME trainer_aircraft_stabilizer_tests COMMAND trainer_aircraft_stabilizer_tests)
    add_test(NAME trainer_aircraft_propeller_tests COMMAND trainer_aircraft_propeller_tests)
    add_test(NAME trainer_aircraft_landing_gear_tests COMMAND trainer_aircraft_landing_gear_tests)
    add_test(NAME trainer_aircraft_main_wing_tests COMMAND trainer_aircraft_main_wing_tests)
    add_test(NAME trainer_aircraft_fuselage_tests COMMAND trainer_aircraft_fuselage_tests)
    add_test(NAME trainer_aircraft_full_aircraft_tests COMMAND trainer_aircraft_full_aircraft_tests)
    add_test(NAME trainer_aircraft_ground_static_trim_tests COMMAND trainer_aircraft_ground_static_trim_tests)
endif()
