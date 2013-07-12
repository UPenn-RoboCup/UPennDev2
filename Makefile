# Master Makefile to compile all Lua/C++ libraries
CWD= $(shell pwd)
PWD= $(subst /,\/,$(CWD)/Player/Lib)
.PHONY: all none clean modules robots

all none: modules robots
	
%:
	@printf "  %b %s\n" Making $@;
	@cd Robots/Transform && make transform && cd $(CWD)
	@cd Robots/$@ && make && cd $(CWD)
	@printf "  %b to %s\n" Installing $(PWD);
	
Webots%:
	@printf "  %b %s %s\n" Making Webots $@;
	@cd Modules/Webots && make && cd $(CWD)
	@cd Robots/Transform && make transform && cd $(CWD)
	@cd Robots/$@ && make && cd $(CWD)
	@printf "  %b for %s\n" Configuring $@;
	@rm -f $(CWD)/Config/Config.lua
	@cd $(CWD)/Config && ln -s Config_$@.lua Config.lua && cd $(CWD)
	@printf "  %b to %s\n" Installing $(PWD);

#naoqi:
#	@echo "Compiling Custom Naoqi Modules...\n"
#	@echo $(PWD)
#	sed -i -e 's/HOME/$(PWD)/g' $(NAOQIDIR)/src/dcmprocess.cpp
#	cd $(NAOQIDIR) && make && cd $(CWD)
#	sed -i -e 's/$(PWD)/HOME/g' $(NAOQIDIR)/src/dcmprocess.cpp
#	@echo "\n"
	
modules:
	@for dir in `ls Modules`; do \
	printf "  %b \n" $$dir ; \
	$(MAKE) -C Modules/$$dir clean; \
	$(MAKE) -C Modules/$$dir; \
	done

robots:
	@for dir in `ls Robots`; do \
	printf "  %b \n" $$dir ; \
	$(MAKE) -C Robots/$$dir clean; \
	$(MAKE) -C Robots/$$dir; \
	done

clean:
	@for dir in `ls Modules`; do \
	printf "  %b \n" $$dir ; \
	$(MAKE) -C Modules/$$dir clean; \
	done
	@for dir in `ls Robots`; do \
	printf "  %b \n" $$dir ; \
	$(MAKE) -C Robots/$$dir clean; \
	done