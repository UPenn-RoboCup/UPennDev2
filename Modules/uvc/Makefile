ifeq ($(shell uname),Darwin)
all:
	@printf "No Mac uvc\n"
clean:
	@printf "No cleaning needed\n"
else
LIBNAME=uvc
EXTRA_OBJ=v4l2.o timeScalar.o
include ../../Makefile.inc
LIBOPTS+=-lm $(LIBRT)
endif

