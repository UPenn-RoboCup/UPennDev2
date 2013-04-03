# Makefile used to build libraries
include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

LIBNAME=NaoCam

ifeq ($(OSTYPE),darwin)
all:
endif

ifeq ($(OSTYPE),linux)
all: NaoCam
endif

# Changed the libraries based on the working NAO branch 
#NaoCam: naoCam.o naoCamThread.o timeScalar.o
NaoCam: nao_cam.o nao_cam_mmap_thread.o cam_util.o timeScalar.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIBRT)

clean:
	rm -f *.$(SHLIBEXT) *.o
