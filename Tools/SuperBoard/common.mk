OSNAME = $(shell uname)

ifeq '$(OSNAME)' 'Linux'
  LINUX=1;
else
  OSX=1;
endif

#MEXEXT = $(shell mexext)
INCLUDES += -I.
CPP_FLAGS = -O2 -fPIC -Wall

all : TARGETS

kBotPacket.o : kBotPacket.c
	g++ -c -o $@ $^ $(CPP_FLAGS) $(INCLUDES)

kBotPacket2.o : kBotPacket2.c
	g++ -c -o $@ $^ $(CPP_FLAGS) $(INCLUDES)

crc32.o : crc32.c
	g++ -c -o $@ $^ $(CPP_FLAGS) $(INCLUDES)

SerialDevice.o : SerialDevice.cc
	g++ -c -o $@ $^ $(CPP_FLAGS) $(INCLUDES)

%.o: %.cc
	g++ -c -o $@ $^ $(CPP_FLAGS) $(INCLUDES)

%.$(MEXEXT): %.cc
	mex -O $^ $(INCLUDES)

%.$(MEXEXT): %.c
	mex -O $^ $(INCLUDES)

clean:
	rm -rf *.o *~

