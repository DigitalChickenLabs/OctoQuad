#!/bin/bash
size_info=$(arm-none-eabi-size "cmake-build-debug/PicoSpiI2cExample.elf" | awk NR\>1); \
text=$(echo "$size_info" | awk '{print $1}'); \
data=$(echo "$size_info" | awk '{print $2}'); \
bss=$(echo "$size_info" | awk '{print $3}'); \
flash=$(($text + $data)); \
sram=$(($bss + $data)); \
echo "FLASH used                    = $flash bytes"; \
echo "SRAM used by global variables = $sram bytes"
