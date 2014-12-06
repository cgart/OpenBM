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
"""TEA Block Encryption Algorithm
"""

from struct import pack
from struct import unpack
import sys

from crypttea._common import _AbstractTEA
from crypttea._common import ULONG_SIZE
from crypttea._common import DELTA


class TEA(_AbstractTEA):
    """TEA block cipher (PEP 272 conform implementation)

    Key: 16 chars or tuple with 4 longs (128 bit)
    blocksize: 8 (64 bit)
    rounds: 32 (default)
    endian: little or big endian (default: host endian)
    IV: initialization vector for OFB (8 chars)
    mode: ECB, OFB
    """

    @property
    def default_rounds(self):
        """Default number of rounds
        """
        return 32

    def _py_decrypt(self, v, k, n=32, delta=DELTA, mask=0xffffffff):
        """Decrypt a quad

        @param v: tuple with (v0, v1)
        @type v: list[2](int)
        @param key: decryption key
        @type key: list[4](int)
        @param n: rounds
        @type n: int
        @return: decrypted values
        @rtype: list[2] int
        """
        v0, v1 = v
        sum = (delta * n) & mask
        for i in range(n):
             v1 -= ((v0<<4) + k[2]) ^ (v0 + sum) ^ ((v0>>5) + k[3])
             v1 &= mask
             v0 -= ((v1<<4) + k[0]) ^ (v1 + sum) ^ ((v1>>5) + k[1])
             v0 &= mask
             sum -= delta
             sum &= mask
        return v0, v1

    def _py_encrypt(self, v, k, n=32, delta=DELTA, mask=0xffffffff):
        """Encrypt a quad

        @param v: tuple with (v0, v1)
        @type v: list[2](int)
        @param key: decryption key
        @type key: list[4](int)
        @param n: rounds
        @type n: int
        @return: decrypted values
        @rtype: list[2] int
        """
        v0=v[0]; v1=v[1]; sum=0
        for i in range(n):
            sum += delta
            sum &= mask
            v0 += ((v1<<4) + k[0]) ^ (v1 + sum) ^ ((v1>>5) + k[1])
            v0 &= mask
            v1 += ((v0<<4) + k[2]) ^ (v0 + sum) ^ ((v0>>5) + k[3])
            v1 &= mask
        return v0, v1

# Try to replace core functions with optimized versions
try:
    from _pyxtea import _c_tea_encrypt
    from _pyxtea import _c_tea_decrypt
except ImportError, msg:
    TEA.setDefaultImpl('py')
    print msg
else:
    TEA._c_encrypt = _c_tea_encrypt
    TEA._c_decrypt = _c_tea_decrypt
    TEA.setDefaultImpl('c')
