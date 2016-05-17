#!/bin/bash

#---------------------------------------------------------------------
# linking the object file created by assembler; no bootloader version.
# on such a version, are used the linker and the crt0i.o from
# the original package, unmodified, and the user program will
# start from the 0x00 address in flash.
#---------------------------------------------------------------------
../bin/gplink -o main.hex -I../share/sdcc/lib/pic16 -I../share/sdcc/non-free/lib/pic16 libio18f46k22.lib libc18f.lib libm18f.lib libdev18f46k22.lib -w -s ../my_sdcc_lib/18f46k22_g.lkr libsdcc.lib main.o
 
