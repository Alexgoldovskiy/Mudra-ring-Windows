.DEFAULT_GOAL := build

COINES_INSTALL_PATH ?= ../COINES_SDK/v2.9.1
TARGET=PC
COINES_BACKEND=COINES_BRIDGE

EXAMPLE_FILE = mudra_ring.c

include $(COINES_INSTALL_PATH)/coines.mk

build: all
	./mudra_ring