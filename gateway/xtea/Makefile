PYTHON?=python2.5
PYTHON24=python2.4
PYTHON25=python2.5
TESTFLAGS=-v
TESTOPTS=
SETUPFLAGS=
KEYID=AD16AB1B

all: inplace

# Build in-place
inplace: 
	$(PYTHON) setup.py $(SETUPFLAGS) build_ext -i

build: 
	$(PYTHON) setup.py $(SETUPFLAGS) build

egg24:
	$(PYTHON24) setup.py $(SETUPFLAGS) bdist_egg

egg25:
	$(PYTHON25) setup.py $(SETUPFLAGS) bdist_egg

dist25: realclean build
	$(PYTHON25) setup.py $(SETUPFLAGS) bdist_egg \
	    sdist --formats=zip,gztar

dist_upload: realclean build
	$(PYTHON25) setup.py $(SETUPFLAGS) bdist_egg \
	    sdist --formats=zip,gztar \
	    upload --sign --identity=$(KEYID)

# What should the default be?
test:
	PYTHONPATH=src $(PYTHON) src/crypttea/tests.py

egg: egg24 egg25

distribution: realclean dist25 egg24 gpg

gpg:
	for F in `find dist -type f -and -not -name '*.asc'`; do \
	    gpg --detach-sign --armor --default-key $(KEYID) $$F; \
	done
	md5sum dist/*.zip dist/*.egg dist/*.gz

clean:
	find . \( -name '*.o' -o -name '*~' -o -name '*.so' -o -name '*.py[cod]' -o -name '*.dll' \) -exec rm -f {} \;
	rm -rf build

realclean: clean
	rm -f TAGS
	rm -rf dist
	$(PYTHON) setup.py clean -a

