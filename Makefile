# Master Makefile to compile all Lua/C++ libraries
CWD= $(shell pwd)
PWD= $(subst /,\/,$(CWD)/Player/Lib)
.PHONY: all none clean modules

all none: modules
	@for dir in `ls Robots`; do \
	printf "  %b \n" $(INFOCOLOR)$$dir$(ENDCOLOR) ; \
	$(MAKE) -C Robots/$$dir; \
	done
	
%:
	@printf "  %b %s\n" $(INFOCOLOR)Making$(ENDCOLOR) $@;
	@cd Robots/Transform && make transform && cd $(CWD)
	@cd Robots/$@ && make && cd $(CWD)
	@printf "  %b to %s\n" $(INFOCOLOR)Installing$(ENDCOLOR) $(PWD);
	
Webots%:
	@printf "  %b %s %s\n" $(INFOCOLOR)Making$(ENDCOLOR) Webots $@;
	@cd Modules/Webots && make && cd $(CWD)
	@cd Robots/Transform && make transform && cd $(CWD)
	@cd Robots/$@ && make && cd $(CWD)
	@printf "  %b for %s\n" $(INFOCOLOR)Configuring$(ENDCOLOR) $@;
	@rm -f $(CWD)/Config/Config.lua
	@cd $(CWD)/Config && ln -s Config_$@.lua Config.lua && cd $(CWD)
	@printf "  %b to %s\n" $(INFOCOLOR)Installing$(ENDCOLOR) $(PWD);

#naoqi:
#	@echo "Compiling Custom Naoqi Modules...\n"
#	@echo $(PWD)
#	sed -i -e 's/HOME/$(PWD)/g' $(NAOQIDIR)/src/dcmprocess.cpp
#	cd $(NAOQIDIR) && make && cd $(CWD)
#	sed -i -e 's/$(PWD)/HOME/g' $(NAOQIDIR)/src/dcmprocess.cpp
#	@echo "\n"
	
modules:
	@for dir in `ls Modules`; do \
	printf "  %b \n" $(INFOCOLOR)$$dir$(ENDCOLOR) ; \
	$(MAKE) -C Modules/$$dir clean; \
	$(MAKE) -C Modules/$$dir; \
	done

clean:
	@for dir in `ls Modules`; do \
	printf "  %b \n" $(INFOCOLOR)$$dir$(ENDCOLOR) ; \
	$(MAKE) -C Modules/$$dir clean; \
	done
	@for dir in `ls Robots`; do \
	printf "  %b \n" $(INFOCOLOR)$$dir$(ENDCOLOR) ; \
	$(MAKE) -C Robots/$$dir clean; \
	done