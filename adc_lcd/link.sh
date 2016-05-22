#!/bin/bash

#---------------------------------------------------------------
# linking the object file created by assembler; results hex file
#---------------------------------------------------------------
../bin/gplink -o main.hex -I../share/sdcc/lib/pic16 -I../share/sdcc/non-free/lib/pic16 libc18f.lib libm18f.lib libdev18f46k22.lib -w -s ../my_sdcc_lib/18f46k22_boot_g.lkr libsdcc.lib main.o  
