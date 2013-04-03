# Makefile used to build libraries
include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

NaoQiSDK=/usr
NaoQi_INC=-I$(NaoQiSDK)/include
NaoQi_LIB=-L$(NaoQiSDK)/lib -lalmemoryfastaccess

INCLUDES=$(NaoQi_INC)

SOURCES=\
			src/actuator_process.c \
			src/dcmprocess.c \
			src/luadcm.c \
			src/luafifo.c \
			src/sensor_process.c \
			src/shmmap.c \
			src/shm_util.c

OBJECTS=$(SOURCES:.c=.o)

all: luadcm

luadcm: $(OBJECTS)
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm -lluajit-5.1 $(NaoQi_LIB)

clean:
	rm -f *.$(SHLIBEXT) *.o


