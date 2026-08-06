.PHONY: print_cflags

# Compatibility target consumed by the micro-ROS static-library builder.
# Firmware configuration and builds continue to use CMake presets.
print_cflags:
	@echo "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -fno-common"
