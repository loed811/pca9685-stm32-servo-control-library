# Tests and build checks for the PCA9685 driver.
#
#   make            build and run the host unit tests (with ASan/UBSan)
#   make arm        compile the driver, the STM32 HAL layer and the examples
#                   for Cortex-M0, M3 and M4F (needs arm-none-eabi-gcc)
#   make clean
#
# Use another compiler with e.g. `make CC=clang`.

ROOT    := ..
BUILD   := build

WARN    := -Wall -Wextra -Wpedantic -Wshadow -Wstrict-prototypes -Werror
# The driver itself must also build cleanly with the stricter embedded set.
LIBWARN := $(WARN) -Wconversion -Wsign-conversion -Wdouble-promotion -Wcast-qual
SAN     := -fsanitize=address,undefined,float-cast-overflow \
           -fno-sanitize-recover=all -fno-omit-frame-pointer
CFLAGS  := -std=c99 -O1 -g $(SAN) -I$(ROOT) -I.

.PHONY: test arm clean

test: $(BUILD)/test_pca9685
	./$(BUILD)/test_pca9685

$(BUILD):
	mkdir -p $@

$(BUILD)/pca9685_i2c.o: $(ROOT)/pca9685_i2c.c $(ROOT)/pca9685_i2c.h $(ROOT)/pca9685_i2c_hal.h | $(BUILD)
	$(CC) $(CFLAGS) $(LIBWARN) -c $< -o $@

$(BUILD)/%.o: %.c mock_hal.h $(ROOT)/pca9685_i2c.h $(ROOT)/pca9685_i2c_hal.h | $(BUILD)
	$(CC) $(CFLAGS) $(WARN) -c $< -o $@

$(BUILD)/test_pca9685: $(BUILD)/pca9685_i2c.o $(BUILD)/mock_hal.o $(BUILD)/test_pca9685.o
	$(CC) $(CFLAGS) $^ -lm -o $@

# --- Cross-compilation check ----------------------------------------------

ARM_CC    ?= arm-none-eabi-gcc
ARM_SIZE  ?= arm-none-eabi-size
ARM_EXTRA ?=
ARM_COMMON = -std=c99 -Os -ffunction-sections -fdata-sections \
             -I$(ROOT) -Istm32_stub $(ARM_EXTRA)

ARM_FLAGS_m0 := -mcpu=cortex-m0 -mthumb
ARM_FLAGS_m3 := -mcpu=cortex-m3 -mthumb
ARM_FLAGS_m4 := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

arm: arm-m0 arm-m3 arm-m4

arm-%: | $(BUILD)
	mkdir -p $(BUILD)/$@
	$(ARM_CC) $(ARM_FLAGS_$*) $(ARM_COMMON) $(LIBWARN) -c $(ROOT)/pca9685_i2c.c -o $(BUILD)/$@/pca9685_i2c.o
	$(ARM_CC) $(ARM_FLAGS_$*) $(ARM_COMMON) $(WARN) -c $(ROOT)/pca9685_i2c_hal.c -o $(BUILD)/$@/pca9685_i2c_hal.o
	$(ARM_CC) $(ARM_FLAGS_$*) $(ARM_COMMON) $(WARN) -DPCA9685_I2C_HANDLE=hi2c2 -c $(ROOT)/pca9685_i2c_hal.c -o $(BUILD)/$@/pca9685_i2c_hal_i2c2.o
	$(ARM_CC) $(ARM_FLAGS_$*) $(ARM_COMMON) $(WARN) -c $(ROOT)/examples/single_servo_sweep/main.c -o $(BUILD)/$@/single_servo_sweep.o
	$(ARM_CC) $(ARM_FLAGS_$*) $(ARM_COMMON) $(WARN) -c $(ROOT)/examples/multi_servo/main.c -o $(BUILD)/$@/multi_servo.o
	-$(ARM_SIZE) $(BUILD)/$@/pca9685_i2c.o $(BUILD)/$@/pca9685_i2c_hal.o

clean:
	rm -rf $(BUILD)
