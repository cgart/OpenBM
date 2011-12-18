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
CC=avr-gcc
CCC=avr-g++
CXX=avr-g++
FC=gfortran
AS=avr-as

# Macros
CND_PLATFORM=AVR-Linux-x86
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/ibus.o \
	${OBJECTDIR}/obm_special.o \
	${OBJECTDIR}/display.o \
	${OBJECTDIR}/buttons.o \
	${OBJECTDIR}/twimaster.o \
	${OBJECTDIR}/power_module.o \
	${OBJECTDIR}/emul_mid.o \
	${OBJECTDIR}/uart.o \
	${OBJECTDIR}/main.o \
	${OBJECTDIR}/photo_sensor.o \
	${OBJECTDIR}/leds.o


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
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/bordmonitor

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/bordmonitor: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.c} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/bordmonitor ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/ibus.o: ibus.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/ibus.o ibus.c

${OBJECTDIR}/obm_special.o: obm_special.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/obm_special.o obm_special.c

${OBJECTDIR}/display.o: display.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/display.o display.c

${OBJECTDIR}/buttons.o: buttons.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/buttons.o buttons.c

${OBJECTDIR}/twimaster.o: twimaster.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/twimaster.o twimaster.c

${OBJECTDIR}/power_module.o: power_module.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/power_module.o power_module.c

${OBJECTDIR}/emul_mid.o: emul_mid.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/emul_mid.o emul_mid.c

${OBJECTDIR}/uart.o: uart.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/uart.o uart.c

${OBJECTDIR}/main.o: main.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/main.o main.c

${OBJECTDIR}/photo_sensor.o: photo_sensor.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/photo_sensor.o photo_sensor.c

${OBJECTDIR}/leds.o: leds.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/leds.o leds.c

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/bordmonitor

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
