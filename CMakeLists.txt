cmake_minimum_required(VERSION 3.21)

project(pca9685_i2c
    VERSION 1.1.0
    DESCRIPTION "Platform-agnostic C driver for the NXP PCA9685 16-channel PWM controller"
    LANGUAGES C)

# Core driver. The HAL functions declared in pca9685_i2c_hal.h are provided
# by the platform port (e.g. port/stm32) or by the test simulator.
add_library(pca9685_i2c STATIC src/pca9685_i2c.c)
target_include_directories(pca9685_i2c PUBLIC include)
target_compile_features(pca9685_i2c PUBLIC c_std_99)

option(PCA9685_BUILD_TESTS "Build the host unit tests" ${PROJECT_IS_TOP_LEVEL})
option(PCA9685_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

if(PROJECT_IS_TOP_LEVEL)
    target_compile_options(pca9685_i2c PRIVATE
        -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wdouble-promotion -Werror)
endif()

if(PCA9685_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

if(PCA9685_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
