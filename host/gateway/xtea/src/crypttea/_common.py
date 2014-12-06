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
"""Common code for all TEA algorithms
"""
from struct import pack
from struct import unpack
import sys

__all__ = ['MODE_ECB', 'MODE_OFB', 'BIG_ENDIAN', 'LITTLE_ENDIAN',
           'NETWORK_ENDIAN', 'HOST_ENDIAN', 'pad']

MODE_ECB = 1 # Electronic Code Book
MODE_CBC = 2 # Cipher Block Chaining
MODE_CFB = 4 # Cipher Feedback 
MODE_OFB = 5 # Output Feedback
MODE_CTR = 6 # Counter

BIG_ENDIAN = 'big'
LITTLE_ENDIAN = 'little'
NETWORK_ENDIAN = LITTLE_ENDIAN
HOST_ENDIAN = sys.byteorder

NULL='\x00'
ULONG_SIZE=4
DELTA=0x9E3779B9L

def pad(s, align, char=NULL):
    """Pad string 's' to 'align' number of elements with char 'char'

    >>> pad('abcd', 4)
    'abcd'
    >>> pad('abcdabcd', 4)
    'abcdabcd'
    >>> pad('abcdef', 4)
    'abcdef\\x00\\x00'
    >>> pad('abcdef', 4, char='0')
    'abcdef00'
    """
    mod = len(s) % align
    if mod != 0:
        return s+mod*char
    else:
        return s

class _AbstractTEA(object):
    """Abstract TEA base class
    """
    @property
    def key_size(self):
        """Size of key
        """
        return 16

    @property
    def block_size(self):
        """Size of blocks
        """
        return 8

    @property
    def default_rounds(self):
        """Default number of rounds
        """
        return 32

    def __init__(self, key, mode=MODE_ECB, IV=8*NULL, rounds=None,
                 endian=HOST_ENDIAN, counter=None, segment_size=None):
        if endian == BIG_ENDIAN:
            self._endian = ">"
        elif endian == LITTLE_ENDIAN:
            self._endian = "<"
        else:
            raise ValueError("Unknown endian: %s" % endian)

        ks = self.key_size
        if isinstance(key, tuple):
            if len(key) != ks/ULONG_SIZE:
                raise ValueError("Invalid key size")
            for e in key:
                if not isinstance(e, (long, int)):
                    raise TypeError("Wrong type %s in key" % repr(e))
            self._key = key
        elif isinstance(key, str):
            if len(key) != ks:
                raise ValueError("Invalid key size")
            self._key = unpack("%s%iL" % (self._endian, ks/ULONG_SIZE), key)
        else:
            raise TypeError("Invalid key type")

        if mode == MODE_ECB:
            pass
        elif mode == MODE_OFB:
            if not isinstance(IV, str) or len(IV) != self.block_size:
                raise ValueError("Invalid IV")
        else:
            raise ValueError("Unknown or unsupported mode")
        self._mode = mode
        self._iv = IV

        if rounds is None:
            rounds = self.default_rounds
        if rounds < 2 or not isinstance(rounds, int):
            raise ValueError("Invalid rounds")
        self._rounds = rounds
        self._counter = counter
        self._segment_size = segment_size

    @classmethod
    def new(cls, key, **kwargs):
        """PEP 272 conform constructor
        """
        return cls(key, **kwargs)

    @classmethod
    def setDefaultImpl(cls, impl='py'):
        """Set default implementation
        """
        if impl == 'py':
            cls._decrypt = cls._py_decrypt
            cls._encrypt = cls._py_encrypt
        elif impl == 'c':
            assert cls._c_decrypt is not None
            cls._decrypt = cls._c_decrypt
            cls._encrypt = cls._c_encrypt
        else:
            raise ValueError(impl)

    def setImpl(self, impl='py'):
        """Set implementation for this instance
        """
        if impl == 'py':
            self._decrypt = self._py_decrypt
            self._encrypt = self._py_encrypt
        elif impl == 'c':
            assert self._c_decrypt is not None
            self._decrypt = self._c_decrypt
            self._encrypt = self._c_encrypt
        else:
            raise ValueError(impl)

    def encrypt(self, block):
        """Encrypt a block

        @param block: block to encrypt
        @type block: str
        """
        if self._mode == MODE_ECB:
            return self._ecb(block, func=self._encrypt)
        elif self._mode == MODE_OFB:
            return self._ofb(block)
        else:
            raise ValueError("Unknown or unsupported mode")

    def decrypt(self, block):
        """Decrypt a block

        @param block: block to decrypt
        @type block: str
        """
        if self._mode == MODE_ECB:
            return self._ecb(block, func=self._decrypt)
        elif self._mode == MODE_OFB:
            return self._ofb(block)
        else:
            raise ValueError("Unknown or unsupported mode")

    _decrypt = None
    _encrypt = None
    _c_decrypt = None
    _c_encrypt = None
    _py_decrypt = None
    _py_encrypt = None

    def _ecb(self, block, func):
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
            raise ValueError(l)
        for i in range(l/bs):
            v = unpack(fmt, block[i*bs:(i+1)*bs])
            w = func(v, key, rounds)
            result.append(pack(fmt, *w))
        return ''.join(result)

    def _ofb_keygen(self):
        """Key generator

        @return: generator function
        """
        key, iv, rounds = self._key, self._iv, self._rounds
        fmt = self._endian+"%iL" % (self.block_size/ULONG_SIZE)
        #v0, v1 = unpack(self._endian+"2L", iv)
        while True:
            v = unpack(fmt, iv)
            w = self._encrypt(v, key, rounds)
            iv = pack(fmt, *w)
            for k in iv:
                yield ord(k)

    def _ofb(self, block):
        """Output Feedback (OFB) encryption requires an IV
        """
        key = pack(self._endian+"%iL" % (self.key_size/ULONG_SIZE), *self._key)
        gen = self._ofb_keygen()
        xor = [chr(x^y) for (x, y) in zip(map(ord, block), gen)]
        return ''.join(xor)

def urshift(a, b):
    mask = 0xffffffffL
    a = abs(a) & mask
    b = abs(b) & mask
    b = b & 0x1f
    return a >> b

def ulshift(a, b):
    mask = 0xffffffffL
    a = abs(a) & mask
    b = abs(b) & mask
    b = b & 0x1f
    return a << b
