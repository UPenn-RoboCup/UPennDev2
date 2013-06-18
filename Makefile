# Master Makefile to compile all Lua/C++ libraries
CWD= $(shell pwd)
PWD= $(subst /,\/,$(CWD)/Player/Lib)
include $(CWD)/Makefile.inc
.PHONY: all none clean modules

all none:
	@echo " Please select following target: "
	@echo " make [PLATFORM]"
	@echo " make naoqi"
	
%:
	$(V)printf "  %b %s\n" $(INFOCOLOR)Making$(ENDCOLOR) $@;
	$(V)cd Platforms/Transform && make transform && cd $(CWD)
	$(V)cd Platforms/$@ && make && cd $(CWD)
	$(V)printf "  %b to %s\n" $(INFOCOLOR)Installing$(ENDCOLOR) $(PWD);
	
Webots%:
	$(V)printf "  %b %s %s\n" $(INFOCOLOR)Making$(ENDCOLOR) Webots $@;
	$(V)cd Modules/Webots && make && cd $(CWD)
	$(V)cd Platforms/Transform && make transform && cd $(CWD)
	$(V)cd Platforms/$@ && make && cd $(CWD)
	$(V)printf "  %b for %s\n" $(INFOCOLOR)Configuring$(ENDCOLOR) $@;
	$(V)rm -f $(CWD)/Config/Config.lua
	$(V)cd $(CWD)/Config && ln -s Config_$@.lua Config.lua && cd $(CWD)
	$(V)printf "  %b to %s\n" $(INFOCOLOR)Installing$(ENDCOLOR) $(PWD);

naoqi:
	@echo "Compiling Custom Naoqi Modules...\n"
	@echo $(PWD)
	sed -i -e 's/HOME/$(PWD)/g' $(NAOQIDIR)/src/dcmprocess.cpp
	cd $(NAOQIDIR) && make && cd $(CWD)
	sed -i -e 's/$(PWD)/HOME/g' $(NAOQIDIR)/src/dcmprocess.cpp
	@echo "\n"
	
modules:
	cd Modules
	for dir in `ls Modules`; do \
		echo $$dir ;\
	$(MAKE) -C Modules/$$dir; \
	done
