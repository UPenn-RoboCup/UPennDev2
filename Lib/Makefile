PLATFORMS = setup_nao setup_naov4 setup_op setup_xos setup_webots_nao setup_webots_op

CMAKE = false
ifeq ($(CMAKE),false)
	CMAKE_BIN= cmake_bin
else
	CMAKE_BIN= cmake 
endif

all none:
	@echo "Please do:"
	@echo "\tmake PLATFORM"
	@echo "where PLATFORM is one of following:"
	@echo "\t$(PLATFORMS)"

$(PLATFORMS):
	@if [ ! -z "$(shell which $(CMAKE_BIN))" ]; then \
			rm -rf build; \
			mkdir build;\
		else \
			make -f Makefile.mk $@; \
		fi

clean:
	@if [ ! -z "$(shell which $(CMAKE_BIN))" ]; then \
		rm -rf build; \
	else \
		make -f Makefile.mk clean; \
	fi
