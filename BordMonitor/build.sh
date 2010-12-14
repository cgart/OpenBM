#!/bin/bash

rm -f customer.h
./scripts/generate_headers.sh $1
cd bootloader
#make -f Makefile all
cd ..
make -f Makefile.custom
#rm -f customer.h
