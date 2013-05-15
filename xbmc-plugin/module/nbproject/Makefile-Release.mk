#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
CND_CONF=Release
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/main.o \
	${OBJECTDIR}/BC.o \
	${OBJECTDIR}/MID.o \
	${OBJECTDIR}/TCPIPClient.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=-pthread
CXXFLAGS=-pthread

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=/usr/lib/libboost_system-mt.a /usr/lib/libboost_thread-mt.a

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libpyBMclient.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libpyBMclient.so: /usr/lib/libboost_system-mt.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libpyBMclient.so: /usr/lib/libboost_thread-mt.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libpyBMclient.so: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -shared -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libpyBMclient.so -s -fPIC ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/main.o: main.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -O2 -Wall -s -I/media/sda6/home/tevs/src/psyBMW_trunk/OpenBM-Gateway -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/main.o main.cpp

${OBJECTDIR}/BC.o: BC.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -O2 -Wall -s -I/media/sda6/home/tevs/src/psyBMW_trunk/OpenBM-Gateway -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/BC.o BC.cpp

${OBJECTDIR}/MID.o: MID.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -O2 -Wall -s -I/media/sda6/home/tevs/src/psyBMW_trunk/OpenBM-Gateway -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/MID.o MID.cpp

${OBJECTDIR}/TCPIPClient.o: TCPIPClient.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -O2 -Wall -s -I/media/sda6/home/tevs/src/psyBMW_trunk/OpenBM-Gateway -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/TCPIPClient.o TCPIPClient.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libpyBMclient.so

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
