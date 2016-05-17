#!/bin/bash

#--------------------------------------------------------------
# it builds the entire project, the version without bootloader.
# on such a version, are used the linker and the crt0i.o from
# the original package, unmodified, and the user program will
# start from the 0x00 address in flash.
#--------------------------------------------------------------
./clean.sh
./compile.sh
./asm.sh
./link_noboot.sh
