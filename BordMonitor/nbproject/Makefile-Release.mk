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
CCADMIN=CCadmin
RANLIB=ranlib
CC=avr-gcc
CCC=avr-g++
CXX=avr-g++
FC=
AS=avr-as

# Macros
CND_PLATFORM=AVR-Linux-x86
CND_CONF=Release
CND_DISTDIR=dist

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=build/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/uart0.o \
	${OBJECTDIR}/main.o

# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-Release.mk dist/Release/AVR-Linux-x86/bordmonitor

dist/Release/AVR-Linux-x86/bordmonitor: ${OBJECTFILES}
	${MKDIR} -p dist/Release/AVR-Linux-x86
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/bordmonitor ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/uart0.o: nbproject/Makefile-${CND_CONF}.mk /media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/uart0.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor
	${RM} $@.d
	$(COMPILE.cc) -O2 -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/uart0.o /media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/uart0.cpp

${OBJECTDIR}/main.o: nbproject/Makefile-${CND_CONF}.mk main.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -O2 -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/main.o main.c

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf:
	${RM} -r build/Release
	${RM} dist/Release/AVR-Linux-x86/bordmonitor

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
