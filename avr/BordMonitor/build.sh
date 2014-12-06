#!/bin/bash

# generate config file including codings for the customer
rm -f customer.h
rm -f firmware.rom
./scripts/generate_headers.sh $1
./scripts/h2py.py include/config.h
#cd bootloader
#make -f Makefile all
#cd ..

# build main project
make -f Makefile.custom

# encrypt firmware with user password (not supported at the moment)
#python bin2rom.py bmwMonitor.out.bin key.pwd

