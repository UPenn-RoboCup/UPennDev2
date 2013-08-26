# Master Makefile to compile all Lua/C++ libraries
CWD= $(shell pwd)
PWD= $(subst /,\/,$(CWD)/Player/Lib)
.PHONY: all none clean transform modules robots gazebo

all none: modules robots

%:
	@cd Robots/Transform && make && cd $(CWD)
	@printf "  %s %s\n" Making $@;
	@cd Robots/$@ && make && cd $(CWD)
	@printf "  %s %s\n" Configuring $@;
	@rm -f $(CWD)/Config/Config.lua
	@ln -s $(CWD)/Config/Config_$@.lua $(CWD)/Config/Config.lua

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

# Must make transform first
robots:
	@cd Robots/Transform && make && cd $(CWD)
	@for dir in `ls Robots`; do \
	printf "  %b \n" $$dir ; \
	$(MAKE) -C Robots/$$dir clean; \
	$(MAKE) -C Robots/$$dir; \
	done

gazebo:
	@for dir in `ls Gazebo`; do \
	printf "  %b \n" $$dir ; \
	$(MAKE) -C Gazebo/$$dir; \
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
	@rm -f $(CWD)/Config/Config.lua