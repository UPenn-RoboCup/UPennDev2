CC=gcc
CPPFLAGS=-c -I/usr/local/include -Iinclude -Iinclude/TH -Isrc/torch \
	-DTH_EXPORTS -DHAVE_MMAP=1 \
	-msse4.2 -DUSE_SSE4_2 -DUSE_SSE3 -DUSE_SSE2 \
	-Wall -Wno-unused-function -Wno-unknown-pragmas \
	-O3 -DNDEBUG -fPIC \
	-DUSE_LAPACK -DUSE_BLAS \
	-FAccelerate
LDFLAGS=-bundle -undefined dynamic_lookup \
	-framework Accelerate \
	-macosx_version_min 10.6 \
	-lm \
	-L/usr/local/lib

SOURCES= \
	src/TH/THGeneral.c \
	src/TH/THStorage.c \
	src/TH/THTensor.c \
	src/TH/THBlas.c \
	src/TH/THLapack.c \
	src/TH/THLogAdd.c \
	src/TH/THRandom.c \
	src/TH/THFile.c \
	src/TH/THDiskFile.c \
	src/TH/THMemoryFile.c \
	src/luaT/luaT.c \
	src/torch/DiskFile.c \
	src/torch/File.c \
	src/torch/MemoryFile.c \
	src/torch/PipeFile.c \
	src/torch/Storage.c \
	src/torch/Tensor.c \
	src/torch/Timer.c \
	src/torch/utils.c \
	src/torch/init.c \
	src/torch/TensorOperator.c \
	src/torch/TensorMath.c \
	src/torch/random.c

OBJECTS=$(SOURCES:.c=.o)
LIBRARY=torch.so
LIBRARY_JIT=torch_jit.so

all: $(SOURCES) $(LIBRARY) $(LIBRARY_JIT)

$(LIBRARY): $(OBJECTS) 
	ld -o $@ $(LDFLAGS) $^ -llua

$(LIBRARY_JIT): $(OBJECTS) 
	ld -o $@ $(LDFLAGS) $^ -lluajit-5.1

.c.o:
	$(CC) $(CPPFLAGS) $< -o $@

clean:
	rm -f $(LIBRARY) 
	rm -f $(OBJECTS)
