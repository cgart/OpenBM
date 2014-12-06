TEA, XTEA and XXTEA Block Encryption Algorithm
==============================================

License: BSD (algorithms are Public Domain)

Authors: Christian Heimes (christian (at) cheimes (dot) de)

Thank you very much to Paul Chakravarti for his initial implementation of
XTEA! Some parts of the package are inspired by his work.

Christian Heimes
----------------

PEP 272 conform implementation of XTEA using the original crypt(),
xtea_decrypt() and xtea_encrypt() functions from Paul Chakravarti.

Python implementation of TEA and XTEA

Pyrex/C optimization for TEA, XTEA and XXTEA

setup.py, Makefile and eggification

unit tests

Benchmarks
==========

Benchmark results on my old laptop (P4 1GHz) on Linux.

TEA
---
c: 5 times 1k data: 0.0532s
c: 5 times 5k data: 0.1423s
c: 5 times 10k data: 0.2663s
c: 5 times 50k data: 1.2416s
py: 5 times 1k data: 0.5844s
py: 5 times 5k data: 2.8673s
py: 5 times 10k data: 5.8496s
py: 5 times 50k data: 28.6454s

XTEA
----
c: 5 times 1k data: 0.0560s
c: 5 times 5k data: 0.1944s
c: 5 times 10k data: 0.2763s
c: 5 times 50k data: 1.2644s
py: 5 times 1k data: 0.6398s
py: 5 times 5k data: 2.9668s
py: 5 times 10k data: 5.9250s
py: 5 times 50k data: 29.7908s

XXTEA
-----
c: 5 times 1k data: 0.0630s
c: 5 times 5k data: 0.1758s
c: 5 times 10k data: 0.3324s
c: 5 times 50k data: 1.5715s
py: 5 times 1k data: 1.3206s
py: 5 times 5k data: 6.4871s
py: 5 times 10k data: 13.0435s
py: 5 times 50k data: 66.8669s
