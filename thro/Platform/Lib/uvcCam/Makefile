# Makefile used to build libraries
include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

LIBNAME= uvcCam

ifeq ($(OSTYPE),darwin)
all:
endif

ifeq ($(OSTYPE),linux)
all: uvcCam mexCam
endif

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

uvcCam: timeScalar.o uvcCam.o v4l2.o
	$(LD) $(LDFLAGS) -o $(LIBNAME).$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm $(LIBRT)

mexCam: mexCam.cpp v4l2.cpp
	mex -O $^

clean:
	rm -f *.$(SHLIBEXT) *.o
