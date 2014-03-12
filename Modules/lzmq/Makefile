LIBNAME=lzmq
EXTRA_OBJ= \
					 lzutils.o \
					 poller.o \
					 zcontext.o \
					 zerror.o \
					 zmsg.o \
					 zpoller.o \
					 zsocket.o
include ../../Makefile.inc
INCLUDES+=-Dllthreads2_EXPORTS
LDFLAGS+=-lzmq

# Note, the following may need to be included
# I don't recommend -llua since they may mess up luajit and more
# This means you cannot nest lua inside python that is inside lua :P
#-llua

# Not sure this is needed (seems it is not wanted...)
#CFLAGS+=`python-config --cflags`
