######################################
# Root Makefile for Athena Firmware
# Builds all three processors (MPU, SPU, TPU)
# in both debug and release configurations.
# POSIX shell (macOS/Linux/Git Bash); needs arm-none-eabi-gcc on PATH or ARM_GCC_PATH set.
######################################

# Build directory for final outputs
BUILD_DIR = build

# Phony targets
.PHONY: all clean mpu-debug mpu-release spu-debug spu-release tpu-debug tpu-release \
        mpu spu tpu debug release help host-test

# Default target - build everything
all: debug release

# Build all debug configurations
debug: mpu-debug spu-debug tpu-debug

# Build all release configurations
release: mpu-release spu-release tpu-release

# Build all configurations for specific processor
mpu: mpu-debug mpu-release
spu: spu-debug spu-release
tpu: tpu-debug tpu-release

######################################
# MPU Targets
######################################
mpu-debug:
	@echo "Building MPU (Debug)..."
# 	@cd MPU && $(MAKE) -f STM32Make.make clean
	@cd MPU && $(MAKE) -f STM32Make.make DEBUG=1
	@mkdir -p "$(BUILD_DIR)"
	@cp "MPU/build/debug/MPU_Firmware.elf" "$(BUILD_DIR)/MPU-debug.elf"
	@echo "MPU Debug build complete: $(BUILD_DIR)/MPU-debug.elf"

mpu-release:
	@echo "Building MPU (Release)..."
	@cd MPU && $(MAKE) -f STM32Make.make clean
	@cd MPU && $(MAKE) -f STM32Make.make DEBUG=0
	@mkdir -p "$(BUILD_DIR)"
	@cp "MPU/build/release/MPU_Firmware.elf" "$(BUILD_DIR)/MPU-release.elf"
	@echo "MPU Release build complete: $(BUILD_DIR)/MPU-release.elf"

######################################
# SPU Targets
######################################
spu-debug:
	@echo "Building SPU (Debug)..."
# 	@cd SPU && $(MAKE) -f STM32Make.make clean
	@cd SPU && $(MAKE) -f STM32Make.make DEBUG=1
	@mkdir -p "$(BUILD_DIR)"
	@cp "SPU/build/debug/SPU_Firmware.elf" "$(BUILD_DIR)/SPU-debug.elf"
	@echo "SPU Debug build complete: $(BUILD_DIR)/SPU-debug.elf"

spu-release:
	@echo "Building SPU (Release)..."
	@cd SPU && $(MAKE) -f STM32Make.make clean
	@cd SPU && $(MAKE) -f STM32Make.make DEBUG=0
	@mkdir -p "$(BUILD_DIR)"
	@cp "SPU/build/release/SPU_Firmware.elf" "$(BUILD_DIR)/SPU-release.elf"
	@echo "SPU Release build complete: $(BUILD_DIR)/SPU-release.elf"

######################################
# TPU Targets
######################################
tpu-debug:
	@echo "Building TPU (Debug)..."
# 	@cd TPU && $(MAKE) -f STM32Make.make clean
	@cd TPU && $(MAKE) -f STM32Make.make DEBUG=1
	@mkdir -p "$(BUILD_DIR)"
	@cp "TPU/build/debug/TPU_Firmware.elf" "$(BUILD_DIR)/TPU-debug.elf"
	@echo "TPU Debug build complete: $(BUILD_DIR)/TPU-debug.elf"

tpu-release:
	@echo "Building TPU (Release)..."
	@cd TPU && $(MAKE) -f STM32Make.make clean
	@cd TPU && $(MAKE) -f STM32Make.make DEBUG=0
	@mkdir -p "$(BUILD_DIR)"
	@cp "TPU/build/release/TPU_Firmware.elf" "$(BUILD_DIR)/TPU-release.elf"
	@echo "TPU Release build complete: $(BUILD_DIR)/TPU-release.elf"

######################################
# Clean Target
######################################
clean:
	@echo "Cleaning all builds..."
	@cd MPU && $(MAKE) -f STM32Make.make clean
	@cd SPU && $(MAKE) -f STM32Make.make clean
	@cd TPU && $(MAKE) -f STM32Make.make clean
	@rm -rf "$(BUILD_DIR)"
	@echo "Clean complete"

######################################
# Help Target
######################################
help:
	@echo Athena Firmware Build System
	@echo ============================
	@echo ""
	@echo Available targets:
	@echo   all           - Build all processors in debug and release (default)
	@echo   debug         - Build all processors in debug mode
	@echo   release       - Build all processors in release mode
	@echo ""
	@echo   mpu           - Build MPU in both debug and release
	@echo   spu           - Build SPU in both debug and release
	@echo   tpu           - Build TPU in both debug and release
	@echo ""
	@echo   mpu-debug     - Build MPU debug configuration
	@echo   mpu-release   - Build MPU release configuration
	@echo   spu-debug     - Build SPU debug configuration
	@echo   spu-release   - Build SPU release configuration
	@echo   tpu-debug     - Build TPU debug configuration
	@echo   tpu-release   - Build TPU release configuration
	@echo ""
	@echo   clean         - Clean all builds
	@echo   help          - Show this help message
	@echo ""
	@echo Output files are placed in: $(BUILD_DIR)/
	@echo   MPU-debug.elf, MPU-release.elf
	@echo   SPU-debug.elf, SPU-release.elf
	@echo   TPU-debug.elf, TPU-release.elf

######################################
# Host self-check of the pure-C modules (link protocol, UBX parser, fusion filter)
# Runs on the dev machine with any C compiler, no toolchain or hardware needed.
######################################
host-test:
	@mkdir -p "$(BUILD_DIR)"
	cc -std=c99 -Wall -Wextra -O1 -I Athena test/test_host.c Athena/athena_link.c Athena/ubx.c Athena/fusion.c -lm -o $(BUILD_DIR)/host_test
	./$(BUILD_DIR)/host_test
