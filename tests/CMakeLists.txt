include(FetchContent)

FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG v2.6.1
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(unity)

add_executable(test_pca9685
    test_pca9685.c
    sim/pca9685_sim.c
    ${PROJECT_SOURCE_DIR}/examples/servo_sweep/servo_sweep.c)

target_include_directories(test_pca9685 PRIVATE
    sim
    ${PROJECT_SOURCE_DIR}/examples/servo_sweep)

target_link_libraries(test_pca9685 PRIVATE pca9685_i2c unity)
target_compile_options(test_pca9685 PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_test(NAME pca9685_unit_tests COMMAND test_pca9685)
