#!/bin/bash

rm -f customer.h
./scripts/generate_headers.sh $1
make -f Makefile.custom
rm -f customer.h
