#!/bin/bash

#-----------------------------------------------
# just compile the C source; results an asm file
#-----------------------------------------------
../bin/sdcc -mpic16 -pPIC18F46K22 --use-non-free --obanksel=9 --opt-code-size --denable-peeps --optimize-cmp --optimize-df -c main.c -I../share/sdcc/include/pic16 -I../share/sdcc/non-free/include -I../my_sdcc_lib 
