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
"""xtea unit tests
"""
__author__ = "Christian Heimes"
__version__ = "$Id: tests.py 216 2007-02-28 12:06:48Z tiran $"
__revision__ = "$Revision: 216 $"

import os
import unittest
from doctest import DocTestSuite
from time import time

from crypttea.tea import TEA
from crypttea.xtea import XTEA
from crypttea.xxtea import XXTEA
from crypttea._common import BIG_ENDIAN, LITTLE_ENDIAN, MODE_ECB


class CommonTest(unittest.TestCase):
    cls = None

    def test_ecb(self):
        key = os.urandom(16)
        block = os.urandom(8)
        for impl in ('c', 'py'):
            for bs in (8, 16, 32, 128):
                block = os.urandom(bs)
                for endian in BIG_ENDIAN, LITTLE_ENDIAN:
                    algo = self.cls(key, mode=MODE_ECB, endian=endian)
                    algo.setImpl(impl)
                    enc = algo.encrypt(block)
                    self.failIfEqual(enc, block)
                    self.failUnlessEqual(len(enc), bs)
                    dec = algo.decrypt(enc)
                    self.failUnlessEqual(dec, block)
                    self.failUnlessEqual(len(dec), bs)

    def test_compare_encrypt(self):
        key = (0, 1, 2, 2)
        v = (42, 23)
        algo = self.cls(key)
        pycrypt = algo._py_encrypt
        ccrypt = algo._c_encrypt
        for n in range(1, 33): # 1..32
            py = pycrypt(v, key, n)
            c = ccrypt(v, key, n)
            self.failUnlessEqual(py[0], c[0], repr((py[0] - c[0], n)))
            self.failUnlessEqual(py[1], c[1], repr((py[1] - c[1], n)))

    def test_compare_decrypt(self):
        key = (0, 1, 2, 2)
        v = (42, 23)
        algo = self.cls(key)
        pycrypt = algo._py_encrypt
        ccrypt = algo._c_encrypt
        for n in range(1, 33): # 1..32
            py = pycrypt(v, key, n)
            c = ccrypt(v, key, n)
            self.failUnlessEqual(py[0], c[0], repr((py[0] - c[0], n)))
            self.failUnlessEqual(py[1], c[1], repr((py[1] - c[1], n)))


class TeaTest(CommonTest):
    cls = TEA

class XTeaTest(CommonTest):
    cls = XTEA

class XXTeaTest(CommonTest):
    cls = XXTEA

    def test_compare_encrypt(self):
        key = (0, 1, 2, 2)
        v = (42, 23)
        algo = self.cls(key)
        pycrypt = algo._py_encrypt
        ccrypt = algo._c_encrypt
        for n in [2]:
            py = pycrypt(v, key, n)
            c = ccrypt(v, key, n)
            self.failUnlessEqual(py[0], c[0], repr((py[0] - c[0], n)))
            self.failUnlessEqual(py[1], c[1], repr((py[1] - c[1], n)))

    def test_compare_decrypt(self):
        key = (0, 1, 2, 2)
        v = (42, 23)
        algo = self.cls(key)
        pycrypt = algo._py_encrypt
        ccrypt = algo._c_encrypt
        for n in [2]:
            py = pycrypt(v, key, n)
            c = ccrypt(v, key, n)
            self.failUnlessEqual(py[0], c[0], repr((py[0] - c[0], n)))
            self.failUnlessEqual(py[1], c[1], repr((py[1] - c[1], n)))

def compare_timing(cls=XTEA):
    name = cls.__name__
    print name
    print len(name) * '-'
    key = os.urandom(16)
    data = os.urandom(1024)
    xtea = cls(key)
    for impl in ('c', 'py'):
        for size in (1, 5, 10, 50):
            d = size*data
            xtea.setImpl(impl)
            start = time()
            for i in range(5):
                enc = xtea.encrypt(d)
                xtea.decrypt(enc)
            diff = time() - start
            print "%s: 5 times %sk data: %0.4fs" % (impl, size, diff)

def test_suite():
    return unittest.TestSuite((
        unittest.makeSuite(TeaTest),
        unittest.makeSuite(XTeaTest),
        unittest.makeSuite(XXTeaTest),
        DocTestSuite('crypttea.tea'),
        DocTestSuite('crypttea.xtea'),
        DocTestSuite('crypttea.xxtea'),
        DocTestSuite('crypttea._common'),
        ))

if __name__ == '__main__':
    #compare_timing(TEA)
    #compare_timing(XTEA)
    #compare_timing(XXTEA)
    unittest.main(defaultTest="test_suite")
