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
"""XXTEA Block Encryption Algorithm
"""

from struct import pack
from struct import unpack

from crypttea._common import _AbstractTEA
from crypttea._common import ULONG_SIZE
from crypttea._common import DELTA

class XXTEA(_AbstractTEA):
    """
    key = "\xA5\x26\x12\x0E\x56\xCD\x4D\xD2\xD8\xFF\xE1\x36\x6E\x9B\x96\x0A"

    >>> from _common import *
    >>> xxtea = XXTEA.new('0123456789012345', mode=MODE_ECB, endian=BIG_ENDIAN)
    >>> isinstance(xxtea, XXTEA)
    True
    >>> z = xxtea.encrypt('ABCDEFGH')

    XXX>>> z.encode('hex')
    XXX'6b49d49d72c7dfe5'

    >>> xxtea.decrypt(z)
    'ABCDEFGH'

    #>>> xxtea = XXTEA.new(16*NULL, mode=MODE_ECB, endian=BIG_ENDIAN)
    #>>> z = xxtea.encrypt(4*16*NULL)
    #>>> z.encode('hex')
    """
    @property
    def key_size(self):
        return 16

    @property
    def block_size(self):
        return 8

    @property
    def default_rounds(self):
        return 2

    def XXX_ecb(self, block, func):
        """Electronic Code Book encryption/decryption

        @type block: str
        @param func: decrypt or encrypt function
        @type func: callable(v0, v1, keys[4], rounds)
        """
        l = len(block)
        bs = self.block_size
        key, rounds = self._key, self._rounds
        fmt = self._endian+"2L"
        result = []
        if l % bs != 0:
            raise ValueError
        for i in range(l/bs):
            v = unpack(fmt, block[i*bs:(i+1)*bs])
            w = func(v, key, rounds)
            result.append(pack(fmt, *w))
        return ''.join(result)

    def _py_decrypt(self, v, k, n=2, delta=DELTA, mask=0xffffffff):
        """Decrypt a block

        @param v: tuple with (v0, v1)
        @type v: list[2](int)
        @param key: decryption key
        @type key: list[4](int)
        @param n: rounds
        @type n: int
        @return: decrypted values
        @rtype: list[2] int
        """
        v = list(v)
        q=6+52/n
        z=v[n-1]
        y=v[0] 
        sum=q*delta
        while sum != 0:
            e = sum>>2 & 3
            for p in range(n-1, 0, -1): # for(p=n-1; p>0; p--)
                z = v[p-1]
                v[p] -= (((z>>5)^(y<<2))+(((y>>3)^(z<<4))^(sum^y))+(k[(p&3)^e]^z))
                v[p] &= mask
                y = v[p]
            p=0
            z = v[n-1]
            v[p] -= (((z>>5)^(y<<2))+(((y>>3)^(z<<4))^(sum^y))+(k[(p&3)^e]^z))
            v[p] &= mask
            y = v[p]
            sum -= delta
            sum &= mask
        return v

    def _py_encrypt(self, v, k, n=2, delta=DELTA, mask=0xffffffff):
        """Encrypt a block

        @param v: tuple with (v0, v1)
        @type v: list[2](int)
        @param key: decryption key
        @type key: list[4](int)
        @param n: rounds
        @type n: int
        @return: decrypted values
        @rtype: list[2] int
        """
        v = list(v)
        q=6+52/n
        z=v[n-1]
        y=v[0]
        sum=0
        for q in range(q-1, -1, -1): # q-1 .. 0, while (q-- > 0)
            sum += delta
            sum &= mask
            e = sum >> 2 & 3
            for p in range(n-1): # for (p=0; p<n-1; p++)
                y = v[p+1]
                v[p] += (((z>>5)^(y<<2))+(((y>>3)^(z<<4))^(sum^y))+(k[(p&3)^e]^z))
                v[p] &= mask
                z = v[p]
            p=n-1
            y = v[0]
            v[p] += (((z>>5)^(y<<2))+(((y>>3)^(z<<4))^(sum^y))+(k[(p&3)^e]^z))
            v[p] &= mask
            z = v[p]
        return v

# Try to replace core functions with optimized versions
try:
    from _pyxtea import _c_xxtea_encrypt
    from _pyxtea import _c_xxtea_decrypt
except ImportError, msg:
    XXTEA.setDefaultImpl('py')
    print msg
else:
    XXTEA._c_encrypt = _c_xxtea_encrypt
    XXTEA._c_decrypt = _c_xxtea_decrypt
    XXTEA.setDefaultImpl('c')
