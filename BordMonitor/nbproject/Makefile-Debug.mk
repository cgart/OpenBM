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
CND_CONF=Debug
CND_DISTDIR=dist

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=build/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/ibus.o \
	${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/main.o \
	${OBJECTDIR}/buttons.o \
	${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/uart.o \
	${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/twimaster.o \
	${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/sha256.o \
	${OBJECTDIR}/bootloader.o \
	${OBJECTDIR}/leds.o \
	${OBJECTDIR}/display.o \
	${OBJECTDIR}/emul_mid.o

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
	${MAKE}  -f nbproject/Makefile-Debug.mk dist/Debug/AVR-Linux-x86/bordmonitor

dist/Debug/AVR-Linux-x86/bordmonitor: ${OBJECTFILES}
	${MKDIR} -p dist/Debug/AVR-Linux-x86
	${LINK.c} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/bordmonitor ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/ibus.o: nbproject/Makefile-${CND_CONF}.mk /media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/ibus.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/ibus.o /media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/ibus.c

${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/main.o: nbproject/Makefile-${CND_CONF}.mk /media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/main.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/main.o /media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/main.c

${OBJECTDIR}/buttons.o: nbproject/Makefile-${CND_CONF}.mk buttons.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/buttons.o buttons.c

${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/uart.o: nbproject/Makefile-${CND_CONF}.mk /media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/uart.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/uart.o /media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/uart.c

${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/twimaster.o: nbproject/Makefile-${CND_CONF}.mk /media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/twimaster.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/twimaster.o /media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/twimaster.c

${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/sha256.o: nbproject/Makefile-${CND_CONF}.mk /media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/sha256.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/sha256.o /media/sda6/home/tevs/src/psyBMW_trunk/avr/BordMonitor/sha256.c

${OBJECTDIR}/bootloader.o: nbproject/Makefile-${CND_CONF}.mk bootloader.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/bootloader.o bootloader.c

${OBJECTDIR}/leds.o: nbproject/Makefile-${CND_CONF}.mk leds.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/leds.o leds.c

${OBJECTDIR}/display.o: nbproject/Makefile-${CND_CONF}.mk display.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/display.o display.c

${OBJECTDIR}/emul_mid.o: nbproject/Makefile-${CND_CONF}.mk emul_mid.c 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.c) -g -I./include -MMD -MP -MF $@.d -o ${OBJECTDIR}/emul_mid.o emul_mid.c

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/Debug
	${RM} dist/Debug/AVR-Linux-x86/bordmonitor

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
