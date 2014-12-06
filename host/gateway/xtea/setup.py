#!/usr/bin/env python
"""TEA, XTEA and XXTEA Block Encryption Algorithms

TODO
"""
__author__ = "Christian Heimes"
__version__ = "$Id: setup.py 203 2007-02-27 14:31:27Z tiran $"
__revision__ = "$Revision: 203 $"

import sys
import os

if sys.version_info < (2,4):
    raise RuntimeError("Python 2.4+ required:\n%s" % sys.version)

# boot strap easy setup
SETUPTOOLS_VERSION = "0.6c1"
from ez_setup import use_setuptools
use_setuptools(version=SETUPTOOLS_VERSION)

# import the rest
from setuptools import setup
from setuptools import find_packages
from setuptools import Extension

VERSION = "0.1"

me = "Christian Heimes"
email = "christian@cheimes.de"

from setuptools import Extension
cxtea = Extension('crypttea._pyxtea',
    ['src/crypttea/_pyxtea.pyx', 'src/crypttea/ctea.c']
    )
ext_modules = []
if os.name == 'posix':
    ext_modules.append(cxtea)

setup_infos = dict(
    name = "crypttea",
    version = VERSION,
    description = __doc__[:__doc__.find('\n')].strip(),
    long_description = '\n'.join([line
                                  for line in __doc__.split('\n')[1:]]),
    author = "Christian Heimes",
    author_email = "christian@cheimes.de",
    maintainer = "Christian Heimes",
    maintainer_email = "christian@cheimes.de",
    url = "http://sourceforge.net/projects/pymoul/",
    download_url= "http://cheeseshop.python.org/pypi/",
    license = "BSD",
    keywords = ["xtea", "tea", "xxtea", "btea", "crypt",
                "encryption", "decryption", "cipher", "crypto"],
    platforms = ['Independent'],
    classifiers = (
        #'Development Status :: 6 - Mature',
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        #'License :: Public Domain',
        'License :: OSI Approved :: BSD'
	'Operating System :: OS Independent'
        'Natural Language :: English',
        'Programming Language :: Python',
        'Topic :: Software Development :: Libraries'
    ),
    ext_modules=ext_modules,
    setup_requires = ["setuptools>="+SETUPTOOLS_VERSION,],
    packages = ['crypttea'],
    package_dir = {'' : 'src'},
    zip_safe = True,
)

setup(**setup_infos)

