CWD=$(shell pwd)

all:
	@mkdir -p build && cd build
	@cd build && cmake .. && make && cd $(CWD)
	@cp build/*.so ../.

clean:
	@rm -rf build
