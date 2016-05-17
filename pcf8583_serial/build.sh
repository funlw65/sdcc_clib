#!/bin/bash

#-----------------------------
# it builds the entire project
#-----------------------------
./clean.sh
./compile.sh
./asm.sh
./link.sh
