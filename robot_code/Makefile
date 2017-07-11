#
# template Makefile for development with libkhepera
#
#
.PHONY: clean


# Modify this!
KTEAM_HOME = ~/khepera4_development

# And maybe these
LIBKHEPERA_ROOT  = ${KTEAM_HOME}/libkhepera-2.1

TARGET_SYSTEM	= khepera-2.6

OE_HOME = /usr/local/khepera4-yocto

KHEPERA_TOOLS = ${OE_HOME}

KTEAM_KERNEL_VERSION = 2.6.24

KTEAM_KERNEL_HOME = ${KHEPERA_TOOLS}/build/tmp/sysroots/overo/usr/src

# And don't touch these
ARCH = armv7-a
CROSS_COMPILE = arm-poky-linux-gnueabi-
PATH := $(PATH):${KHEPERA_TOOLS}/build/tmp/sysroots/i686-linux/usr/bin/armv7a-vfp-neon-poky-linux-gnueabi

CC = ${CROSS_COMPILE}gcc
CXX = ${CROSS_COMPILE}g++ 
LD = ${CROSS_COMPILE}ld
AR = ${CROSS_COMPILE}ar
AS = ${CROSS_COMPILE}as

INCPATH = ${LIBKHEPERA_ROOT}/build-${TARGET_SYSTEM}/include

LIBPATH = ${LIBKHEPERA_ROOT}/build-${TARGET_SYSTEM}/lib


# Pointer to the libkhepera build directory
LIBKHEPERA = ${LIBKHEPERA_ROOT}/build-${TARGET_SYSTEM}


SRCS	= $(wildcard *.cpp)
OBJS	= $(patsubst %.c,%.o,${SRCS})
INCS	= -I ${LIBKHEPERA}/include -I FK/inc -I FK/src -I FMLAB/src
LIBS	= -L ${LIBKHEPERA}/lib -lkhepera -lpthread

CFLAGS 	= -O2
CXXFLAGS = -O2 -std=c++11 -Wall -W -fPIC
#for debugging
CFLAGS 	= -g

TARGET	= main main-static

.PHONY: all clean depend

main: main.o 
	@echo "Building $@"
	$(CXX) -o $@ $? $(LIBS) $(INCS) $(CXXFLAGS)

main-static: main.o
	@echo "Building $@"
	@$(CXX) -o $@ $? $(LIBS) -static $(INCS) $(CXXFLAGS)

all: 	${TARGET}

clean : 
	@echo "Cleaning"
	@rm -f *.o .depend ${TARGET} *~

depend:	
	@echo "Building dependencies"
	@rm -f .depend
	@touch .depend
	@makedepend ${SYS_INCLUDES} ${INCS} -Y -f .depend ${SRCS}

%.o:	%.cc
	@echo "Compiling $@"
	$(CXX) $(INCS) -c $(CXXFLAGS) $< -o $@

%.o:	%.cpp
	@echo "Compiling $@"
	$(CXX) $(INCS) -c $(CXXFLAGS) $< -o $@

ifeq (.depend,$(wildcard .depend))
include .depend 
endif
 
