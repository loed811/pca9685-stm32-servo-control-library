# Host-side unit tests for the PCA9685 driver.
#
#   make -C tests            build and run all tests
#   make -C tests STRICT=1   also fail on known (documented) driver bugs
#   make -C tests clean

CC      ?= gcc
SRC_DIR := ..
BUILD   := build

# -O0 keeps the uninitialised-byte checks (poison_stack) meaningful.
CFLAGS  ?= -O0 -g
override CFLAGS += -std=gnu11 -Wall -Wextra -I$(SRC_DIR) -I.
LDLIBS  += -lm

ifeq ($(STRICT),1)
override CFLAGS += -DSTRICT_KNOWN_BUGS
endif

TESTS := $(BUILD)/test_pca9685_i2c $(BUILD)/test_pca9685_i2c_hal_stm32

.PHONY: all test clean
all: test

test: $(TESTS)
	@status=0; for t in $(TESTS); do ./$$t || status=1; echo; done; exit $$status

$(BUILD):
	mkdir -p $@

$(BUILD)/test_pca9685_i2c: test_pca9685_i2c.c fake_pca9685_hal.c $(SRC_DIR)/pca9685_i2c.c \
		test_framework.h fake_pca9685_hal.h $(SRC_DIR)/pca9685_i2c.h $(SRC_DIR)/pca9685_i2c_hal.h | $(BUILD)
	$(CC) $(CFLAGS) -o $@ $(filter %.c,$^) $(LDLIBS)

$(BUILD)/test_pca9685_i2c_hal_stm32: test_pca9685_i2c_hal_stm32.c $(SRC_DIR)/pca9685_i2c_hal.c $(SRC_DIR)/pca9685_i2c.c \
		test_framework.h stubs/main.h $(SRC_DIR)/pca9685_i2c.h $(SRC_DIR)/pca9685_i2c_hal.h | $(BUILD)
	$(CC) $(CFLAGS) -Istubs -o $@ $(filter %.c,$^) $(LDLIBS)

clean:
	rm -rf $(BUILD)
