# Copyright (c) 2007, Christian Heimes
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the Christian Heimes nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""pyrexc optimized version

http://www.python.org/dev/peps/pep-0272/
"""

ctypedef unsigned long ULong
ctypedef unsigned int UInt

cdef extern from "stdlib.h":
    ctypedef unsigned long size_t
    void *malloc(size_t size)
    void free(void* ptr)
    size_t sizeof(void *)

cdef extern from "ctea.h":
    int btea_decipher(ULong *v, ULong *k, UInt n)
    int btea_encipher(ULong *v, ULong *k, UInt n)
    int tea_decipher(ULong *v, ULong *k, UInt n)
    int tea_encipher(ULong *v, ULong *k, UInt n)
    int xtea_decipher(ULong *v, ULong *k, UInt n)
    int xtea_encipher(ULong *v, ULong *k, UInt n)
    int xxtea_decipher(ULong *v, ULong *k, UInt n)
    int xxtea_encipher(ULong *v, ULong *k, UInt n)

# *** BTEA ***
def _c_btea_encrypt(object block, object key, UInt rounds):
    raise NotImplementedError

def _c_btea_decrypt(object block, object key, UInt rounds):
    cdef ULong v[2]
    cdef ULong k[4]
    if rounds != 2:
        raise ValueError('rounds')
    v[0] = block[0]; v[1] = block[1]
    k[0] = key[0]; k[1] = key[1]; k[2] = key[2]; k[3] = key[3]
    rc = btea_decipher(v, k, rounds)
    result = [v[0], v[1]]
    return result

# *** TEA ***
def _c_tea_encrypt(object block, object key, UInt rounds):
    cdef ULong v[2]
    cdef ULong k[4]
    v[0] = block[0]; v[1] = block[1]
    k[0] = key[0]; k[1] = key[1]; k[2] = key[2]; k[3] = key[3]
    rc = tea_encipher(v, k, rounds)
    result = [v[0], v[1]]
    return result

def _c_tea_decrypt(object block, object key, UInt rounds):
    cdef ULong v[2]
    cdef ULong k[4]
    v[0] = block[0]; v[1] = block[1]
    k[0] = key[0]; k[1] = key[1]; k[2] = key[2]; k[3] = key[3]
    rc = tea_decipher(v, k, rounds)
    result = [v[0], v[1]]
    return result

# *** XTEA ***
def _c_xtea_encrypt(object block, object key, UInt rounds):
    cdef ULong v[2]
    cdef ULong k[4]
    v[0] = block[0]; v[1] = block[1]
    k[0] = key[0]; k[1] = key[1]; k[2] = key[2]; k[3] = key[3]
    rc = xtea_encipher(v, k, rounds)
    result = [v[0], v[1]]
    return result

def _c_xtea_decrypt(object block, object key, UInt rounds):
    cdef ULong v[2]
    cdef ULong k[4]
    v[0] = block[0]; v[1] = block[1]
    k[0] = key[0]; k[1] = key[1]; k[2] = key[2]; k[3] = key[3]
    rc = xtea_decipher(v, k, rounds)
    result = [v[0], v[1]]
    return result

# *** XXTEA ***
def _c_xxtea_encrypt(object block, key, UInt rounds):
    cdef ULong k[4], *v
    cdef int rc, size, i
    size = len(block)
    if size != rounds:
        raise ValueError, "len(block) %i != rounds %i" % (size, rounds)
    v = <ULong *>malloc(sizeof(ULong) * size)
    for i from 0 <= i < size:
        v[i] = block[i]
    k[0] = key[0]; k[1] = key[1]; k[2] = key[2]; k[3] = key[3]
    rc = xxtea_encipher(v, k, rounds)
    result = []
    for i from 0 <= i < size:
        result.append(v[i])
    free(v)
    return result

def _c_xxtea_decrypt(object block, key, UInt rounds):
    cdef ULong k[4], *v
    cdef int rc, size, i
    size = len(block)
    if size != rounds:
        raise ValueError, "len(block) %i != rounds %i" % (size, rounds)
    v = <ULong *>malloc(sizeof(ULong) * size)
    for i from 0 <= i < size:
        v[i] = block[i]
    k[0] = key[0]; k[1] = key[1]; k[2] = key[2]; k[3] = key[3]
    rc = xxtea_decipher(v, k, rounds)
    result = []
    for i from 0 <= i < size:
        result.append(v[i])
    free(v)
    return result
