ifneq ($(shell uname),Darwin)
all:
	@printf "No Mac uvc\n"
clean:
	@printf "No cleaning needed\n"
else
LIBNAME=uvc2
#EXTRA_OBJ=v4l2.o timeScalar.o
include ../../Makefile.inc
LDFLAGS+=-lm $(LIBRT) -luvc
endif

