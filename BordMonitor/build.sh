#!/bin/bash

rm -f customer.h
rm -f firmware.rom
./scripts/generate_headers.sh $1
./scripts/h2py.py include/config.h
cd bootloader
make -f Makefile all
cd ..
make -f Makefile.custom
#rm -f customer.h
python bin2rom.py bmwMonitor.out.bin key.pwd

