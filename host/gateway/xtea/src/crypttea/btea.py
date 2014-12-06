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
"""BTEA Block Decryption Algorithm
"""

from struct import pack
from struct import unpack

from crypttea._common import _AbstractTEA
from crypttea._common import ULONG_SIZE
from crypttea._common import DELTA

class BTEA(_AbstractTEA):
    """
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
        num = 2
        key = (52 / num + 6) * DELTA
        while key != 0:
            xorkey = (key >> 2) & 3
            numloop = num - 1
            while numloop != 0:
                v[numloop] -= ((((v[numloop - 1] << 4) ^ (v[numloop - 1] >> 3)) +
                    ((v[numloop - 1] >> 5) ^ (v[numloop - 1] << 2))) ^
                    ((k[(numloop & 3) ^ xorkey] ^ v[numloop - 1]) +
                    (key ^ v[numloop - 1])))
                v[numloop] &= 0xffffffff
                numloop -= 1
            v[0] -= ((((v[num - 1] << 4) ^ (v[num - 1] >> 3)) +
                ((v[num - 1] >> 5) ^ (v[num - 1] << 2))) ^
                ((k[(0 & 3) ^ xorkey] ^ v[num - 1]) +
                (key ^ v[num - 1])))
            v[0] &= 0xffffffff
            key -= 0x9E3779B9
        return v

    def _py_encrypt(self, v, k, n=32, delta=DELTA, mask=0xffffffff):
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
        raise NotImplementedError

# Try to replace core functions with optimized versions
try:
    from _pyxtea import _c_btea_encrypt
    from _pyxtea import _c_btea_decrypt
except ImportError, msg:
    BTEA.setDefaultImpl('py')
    print msg
else:
    BTEA._c_encrypt = _c_btea_encrypt
    BTEA._c_decrypt = _c_btea_decrypt
    BTEA.setDefaultImpl('c')
