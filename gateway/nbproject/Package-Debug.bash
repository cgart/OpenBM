#!/bin/bash -x

#
# Generated - do not edit!
#

# Macros
TOP=`pwd`
CND_PLATFORM=GNU-Linux-x86
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build
NBTMPDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}/tmp-packaging
TMPDIRNAME=tmp-packaging
OUTPUT_PATH=${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/openbm-gateway
OUTPUT_BASENAME=openbm-gateway
PACKAGE_TOP_DIR=/usr/

# Functions
function checkReturnCode
{
    rc=$?
    if [ $rc != 0 ]
    then
        exit $rc
    fi
}
function makeDirectory
# $1 directory path
# $2 permission (optional)
{
    mkdir -p "$1"
    checkReturnCode
    if [ "$2" != "" ]
    then
      chmod $2 "$1"
      checkReturnCode
    fi
}
function copyFileToTmpDir
# $1 from-file path
# $2 to-file path
# $3 permission
{
    cp "$1" "$2"
    checkReturnCode
    if [ "$3" != "" ]
    then
        chmod $3 "$2"
        checkReturnCode
    fi
}

# Setup
cd "${TOP}"
mkdir -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/package
rm -rf ${NBTMPDIR}
mkdir -p ${NBTMPDIR}

# Copy files and create directories and links
cd "${TOP}"
makeDirectory "${NBTMPDIR}//usr/bin"
copyFileToTmpDir "${OUTPUT_PATH}" "${NBTMPDIR}/${PACKAGE_TOP_DIR}bin/${OUTPUT_BASENAME}" 0755

cd "${TOP}"
makeDirectory "${NBTMPDIR}//usr/share/openbm"
copyFileToTmpDir "key-xbmc.event" "${NBTMPDIR}/${PACKAGE_TOP_DIR}share/openbm/keys-xbmc.event" 0644


# Create control file
cd "${TOP}"
CONTROL_FILE=${NBTMPDIR}/DEBIAN/control
rm -f ${CONTROL_FILE}
mkdir -p ${NBTMPDIR}/DEBIAN

cd "${TOP}"
echo 'Package: openbm-gateway' >> ${CONTROL_FILE}
echo 'Version: 1.2' >> ${CONTROL_FILE}
echo 'Architecture: i386' >> ${CONTROL_FILE}
echo 'Maintainer: Art Tevs <art@tevs.eu>' >> ${CONTROL_FILE}
echo 'Depends: libstdc++6' >> ${CONTROL_FILE}
echo 'Description: daemon acting as a gateway between BMW IBus (hardware) and network clients' >> ${CONTROL_FILE}
echo ''`: '' `' OpenBM-Gateway is a daemon which connects hardware IBus with an network clients over a TCP/IP stack. IBus is preliminary found on cars built between 1995 and 2005 by BMW as also Mini and Range Rover. IBus should be connected through an interface on a serial port which must support hardware collision detection by CTS signals indication. The daemon acts like a gateway server allowing clients to connect to the IBus through a very simple and leightweight TCP/IP protocol. Messages sent over IBus are passed to all connected clients and vice versa.  The main application field is for CarPC installations.' >> ${CONTROL_FILE}

# Create Debian Package
cd "${TOP}"
cd "${NBTMPDIR}/.."
dpkg-deb  --build ${TMPDIRNAME}
checkReturnCode
cd "${TOP}"
mkdir -p  ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/package
mv ${NBTMPDIR}.deb ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/package/openbm-gateway-i386.deb
checkReturnCode
echo Debian: ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/package/openbm-gateway-i386.deb

# Cleanup
cd "${TOP}"
rm -rf ${NBTMPDIR}
