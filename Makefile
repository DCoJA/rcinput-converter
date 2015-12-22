# Makefile for RC input conveter application

PROJECT = rcconv

LDSCRIPT= rcconv.ld
CSRC = rcconv.c

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = cortex-m0
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DHAVE_SYS_H -DFREE_STANDING -DPROCESS_CPPM
OPT   = -O3 -Os -g
LIBS  =

####################
include ./rules.mk

distclean: clean

