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
"""XTEA Block Encryption Algorithm
"""

from struct import pack
from struct import unpack
import sys

from crypttea._common import _AbstractTEA
from crypttea._common import ULONG_SIZE
from crypttea._common import DELTA

class XTEA(_AbstractTEA):
    """XTEA block cipher (PEP 272 conform implementation)

    Key: 16 chars or tuple with 4 longs (128 bit)
    blocksize: 8 (64 bit)
    rounds: 32 (default)
    endian: little or big endian (default: host endian)
    IV: initialization vector for OFB (8 chars)
    mode: ECB, OFB

    >>> from _common import *

    ECB mode encryption and decryption
    >>> xtea = XTEA.new('0123456789012345', mode=MODE_ECB, endian=LITTLE_ENDIAN)
    >>> z = xtea.encrypt('ABCDEFGH')
    >>> z.encode('hex')
    'ea0c3d7c1c22557f'
    >>> xtea.decrypt(z)
    'ABCDEFGH'

    The block must be 8 characters or a multitude of 8 long
    >>> z = xtea.encrypt('ABCDEFGHABCDEFGH')
    >>> z.encode('hex')
    'ea0c3d7c1c22557fea0c3d7c1c22557f'
    >>> xtea.decrypt(z)
    'ABCDEFGHABCDEFGH'

    You can specify the endianes of the key and data. The default value is host endian.
    >>> xtea = XTEA.new('0123456789012345', mode=MODE_ECB, endian=BIG_ENDIAN)
    >>> z = xtea.encrypt('ABCDEFGH')
    >>> z.encode('hex')
    'b67c01662ff6964a'
    >>> xtea.decrypt(z)
    'ABCDEFGH'

    In OFB mode the data block can have any size.
    >>> import os
    >>> key = os.urandom(16)
    >>> iv = os.urandom(8)
    >>> data = os.urandom(10000)
    >>> xtea = XTEA.new(key, mode=MODE_OFB, IV=iv, endian=LITTLE_ENDIAN)
    >>> z = xtea.encrypt(data)
    >>> xtea.decrypt(z) == data
    True

    >>> key = '0123456789012345'
    >>> iv = 'ABCDEFGH'
    >>> xtea = XTEA.new(key, mode=MODE_OFB, IV=iv, endian=BIG_ENDIAN)
    >>> z = xtea.encrypt('Hello There!')
    >>> z.encode('hex')
    'fe196d0a40d6c222b9eff3e9'
    >>> xtea.decrypt(z)
    'Hello There!'
    """
    @property
    def default_rounds(self):
        """Default number of rounds
        """
        return 32

    def _py_decrypt(self, v, key, n=32, delta=DELTA, mask=0xffffffffL):
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
        for round in range(n):
            v1 -= ((v0<<4 ^ v0>>5) + v0) ^ (sum + key[sum>>11 & 3])
            v1 &= mask
            sum -= delta
            sum &= mask
            v0 -= ((v1<<4 ^ v1>>5) + v1) ^ (sum + key[sum & 3])
            v0 &= mask
        return v0, v1

    def _py_encrypt(self, v, key, n=32, delta=DELTA, mask=0xffffffff):
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
        v0, v1 = v
        sum = 0
        for round in range(n):
            v0 += ((v1<<4 ^ v1>>5) + v1) ^ (sum + key[sum & 3])
            v0 &= mask
            sum += delta
            sum &= mask
            v1 += ((v0<<4 ^ v0>>5) + v0) ^ (sum + key[sum>>11 & 3])
            v1 &= mask
        return v0, v1

# Try to replace core functions with optimized versions
try:
    from _pyxtea import _c_xtea_encrypt
    from _pyxtea import _c_xtea_decrypt
except ImportError, msg:
    XTEA.setDefaultImpl('py')
    print msg
else:
    XTEA._c_encrypt = _c_xtea_encrypt
    XTEA._c_decrypt = _c_xtea_decrypt
    XTEA.setDefaultImpl('c')
