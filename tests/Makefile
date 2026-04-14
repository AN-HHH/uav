##############################################################################
# Makefile — host-side unit tests for the UAVCAN-to-CAN bridge
#
# Uses the native C compiler (no STM32 toolchain required).
#
# Usage:
#   make          — build and run all tests
#   make clean    — remove test binaries
##############################################################################

CC     := gcc
CFLAGS := -std=c11 -Wall -Wextra -Wpedantic \
          -I../firmware/Core/Inc \
          -DUNIT_TEST \
          -g -O0

.PHONY: all clean

all: test_uavcan_esc test_can_bridge
	@echo ""
	@echo "--- Running test_uavcan_esc ---"
	./test_uavcan_esc
	@echo ""
	@echo "--- Running test_can_bridge ---"
	./test_can_bridge

test_uavcan_esc: test_uavcan_esc.c
	$(CC) $(CFLAGS) $< -o $@

test_can_bridge: test_can_bridge.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f test_uavcan_esc test_can_bridge
