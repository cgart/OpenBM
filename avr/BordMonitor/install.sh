#!/bin/bash

rm -f customer.h
./scripts/generate_headers.sh $1
make -f Makefile.custom install
rm -f customer.h
