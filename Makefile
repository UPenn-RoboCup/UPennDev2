# Master Makefile to compile all Lua/C++ libraries
CWD= $(shell pwd)
PWD= $(subst /,\/,$(CWD)/Player/Lib)
include $(CWD)/Makefile.inc

all none:
	@echo " Please select following target: "
	@echo " make op"
	@echo " make nao3"
	@echo " make nao"
	@echo " make xos"
	@echo " make charli"
	@echo " make webots_op"
	@echo " make webots_nao"
	@echo " make webots_charli"
	
# TODO: If contains "webots" then also build webots
%:
	$(V)printf "  %b %s\n" $(INFOCOLOR)Making$(ENDCOLOR) $@;
	$(V)cd Platforms/Transform && make transform && cd $(CWD)
	$(V)cd Platforms/$@ && make && cd $(CWD)
	$(V)printf "  %b to %s\n" $(INFOCOLOR)Installing$(ENDCOLOR) $(PWD);
	

webots_%:
	$(V)printf "  %b %s %s\n" $(INFOCOLOR)Making$(ENDCOLOR) Webots $@;
	$(V)cd Frameworks/Webots && make && cd $(CWD)
	$(V)cd Platforms/Transform && make transform && cd $(CWD)
	$(V)cd Platforms/$@ && make && cd $(CWD)

naoqi:
	@echo "Compiling Custom Naoqi Modules...\n"
	@echo $(PWD)
	sed -i -e 's/HOME/$(PWD)/g' $(NAOQIDIR)/src/dcmprocess.cpp
	cd $(NAOQIDIR) && make && cd $(CWD)
	sed -i -e 's/$(PWD)/HOME/g' $(NAOQIDIR)/src/dcmprocess.cpp
	@echo "\n"
	
setup_nao: nao util image comm
	@echo "Setting up Nao Lua/C++ Libraries...\n"
	mkdir -p $(INSTDIR)
	find $(NAODIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	rm -f $(PLAYERDIR)/Config/Config.lua
	cd $(PLAYERDIR)/Config && ln -s Config_Nao.lua Config.lua && cd $(CWD)
	find $(UTILDIR) $(REGEX) -exec cp -v {} $(NAODEPLUA51DIR) \;
	rm -f $(NAODEPLUA51DIR)/*;
	@echo "\n"

# Removed naoqi at the end to compile on laptop
setup_naov4: naov4 util image comm
	@echo "Setting up Nao V4 Lua/C++ Libraries...\n"
#	mkdir -p $(INSTDIR)
#	find $(NAOV4DIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	rm -f $(PLAYERDIR)/Config/Config.lua
	cd $(PLAYERDIR)/Config && ln -s Config_NaoV4.lua Config.lua && cd $(CWD)
	@echo "\n"

setup_webots_nao: webots_nao util image setup_webots occmap
	@echo "Setting up Webots Lua/C++ Libraries...\n"
	mkdir -p $(INSTDIR)
	find $(WEBOTSNAODIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	rm -f $(PLAYERDIR)/Config/Config.lua
	cd $(PLAYERDIR)/Config && ln -s Config_WebotsNao.lua Config.lua && cd $(CWD)
	@echo "\n"

setup_webots_op: webots_op util image setup_webots occmap
	@echo "Setting up Webots Lua/C++ Libraries...\n"
#	mkdir -p $(INSTDIR)
#	find $(WEBOTSOPDIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	rm -f $(PLAYERDIR)/Config/Config.lua
	cd $(PLAYERDIR)/Config && ln -s Config_WebotsOP.lua Config.lua && cd $(CWD)
	@echo "\n"

setup_webots_charli: webots_charli util image setup_webots occmap
	@echo "Setting up Webots Lua/C++ Libraries...\n"
	mkdir -p $(INSTDIR)
	find $(WEBOTSCHARLIDIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	rm -f $(PLAYERDIR)/Config/Config.lua
	cd $(PLAYERDIR)/Config && ln -s Config_WebotsCharli.lua Config.lua && cd $(CWD)
	@echo "\n"

setup_webots_saffir: webots_saffir util image setup_webots
	@echo "Setting up Webots Lua/C++ Libraries...\n"
	mkdir -p $(INSTDIR)
	find $(WEBOTSSAFFIRDIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	rm -f $(PLAYERDIR)/Config/Config.lua
	cd $(PLAYERDIR)/Config && ln -s Config_WebotsSaffir.lua Config.lua && cd $(CWD)
	@echo "\n"

setup_webots_thorop: webots_thorop util image setup_webots
	@echo "Setting up Webots Lua/C++ Libraries...\n"
	mkdir -p $(INSTDIR)
	find $(WEBOTSTHOROPDIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	rm -f $(PLAYERDIR)/Config/Config.lua
	cd $(PLAYERDIR)/Config && ln -s Config_WebotsTHOROP.lua Config.lua && cd $(CWD)
	@echo "\n"

setup_webots_atlas: webots_atlas util image setup_webots
	@echo "Setting up Webots Lua/C++ Libraries...\n"
	mkdir -p $(INSTDIR)
	find $(WEBOTSATLASDIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	rm -f $(PLAYERDIR)/Config/Config.lua
	cd $(PLAYERDIR)/Config && ln -s Config_WebotsAtlas.lua Config.lua && cd $(CWD)
	@echo "\n"

setup_visiontest: visiontest util image 
	@echo "Setting up Darwin OP Lua/C++ Libraries...\n"
	mkdir -p $(INSTDIR)
	find $(VISIONTESTDIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	rm -f $(PLAYERDIR)/Config/Config.lua
	cd $(PLAYERDIR)/Config && ln -s Config_VisionTest.lua Config.lua && cd $(CWD)
	@echo "\n"

setup_op: op util image comm occmap
	@echo "Setting up Darwin OP Lua/C++ Libraries...\n"
#	mkdir -p $(INSTDIR)
#	find $(OPDIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	rm -f $(PLAYERDIR)/Config/Config.lua
	cd $(PLAYERDIR)/Config && ln -s Config_OP.lua Config.lua && cd $(CWD)
	@echo "\n"

setup_xos: xos util image comm
	@echo "Setting up Darwin XOS Lua/C++ Libraries...\n"
	mkdir -p $(INSTDIR)
	find $(XOSDIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	rm -f $(PLAYERDIR)/Config/Config.lua
	cd $(PLAYERDIR)/Config && ln -s Config_XOS.lua Config.lua && cd $(CWD)
	@echo "\n"

setup_charli: charli util image comm hokuyo
	@echo "Setting up Charli Lua/C++ Libraries...\n"
	mkdir -p $(INSTDIR)
	find $(CHARLIDIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	rm -f $(PLAYERDIR)/Config/Config.lua
	cd $(PLAYERDIR)/Config && ln -s Config_Charli.lua Config.lua && cd $(CWD)
	@echo "\n"

setup_webots_cont: webot
	@echo "Setting up WebotsController Lua/C++ Libraries...\n"
	mkdir -p $(WEBOTSCONTDIR)
	find $(WEBOTSDIR)/Controller $(REGEX) -exec cp -v {} $(WEBOTSCONTDIR) \;
	cd $(WEBOTSCONTDIR) && ln -sf ../$(PLAYERDIR) Run && cd $(CWD)
	cd $(WEBOTSCONTDIR) && ln -sf lua_set.sh nao_team_0 && cd $(CWD)
	cd $(WEBOTSCONTDIR) && ln -sf lua_set.sh nao_team_1 && cd $(CWD)
	cd $(WEBOTSCONTDIR) && ln -sf lua_set.sh darwin-op_team_0 && cd $(CWD)
	cd $(WEBOTSCONTDIR) && ln -sf lua_set.sh darwin-op_team_1 && cd $(CWD)
	@echo "\n"

setup_webots: setup_webots_cont

setup_slamT: torch SlamT
	@echo "Setting up Slam Torch/Lua/C++ Libraries...\n"
	mkdir -p $(INSTDIR)
	find Modules/SlamT $(REGEX) -exec cp -v {} $(INSTDIR) \;
	@echo "\n"

setup_primesense: primesense util
	@echo "Setting up PrimeSense Lua/C++ Libraries...\n"
	mkdir -p $(INSTDIR)
	find $(PRIMEDIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	cp $(PRIMEDIR)/SamplesConfig.xml $(INSTDIR)
	rm -f $(PLAYERDIR)/Config/Config.lua
	cd $(PLAYERDIR)/Config && ln -s Config_WebotsOP.lua Config.lua && cd $(CWD)	
	@echo "\n"

setup_hands: hands util
	@echo "Setting up Hands Lua/C++ Libraries...\n"
	mkdir -p $(INSTDIR)
	find $(HANDSDIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	@echo "\n"

setup_naoqi: naoqi util
	@echo "Setting up NaoQi module...\n"
	mkdir -p $(INSTDIR)
	find $(NAOQIDIR) $(REGEX) -exec cp -v {} $(INSTDIR) \;
	sed -i -e 's/HOME/$(PWD)/g' $(INSTDIR)/nao_init.lua
	@echo "\n"
	
clean:
	cd $(IMAGEDIR) && make clean && cd $(CWD)
	cd $(COMMDIR) && make clean && cd $(CWD)
	cd $(HOKUYODIR) && make clean && cd $(CWD)
	cd $(UTILDIR) && make clean && cd $(CWD)
	cd $(PRIMEDIR) && make clean && cd $(CWD)
	cd $(WEBOTSDIR) && make clean && cd $(CWD)
	cd $(NAODIR) && make clean && cd $(CWD)
	cd $(NAOV4DIR) && make clean && cd $(CWD)
	cd $(NAOQIDIR) && make clean && cd $(CWD)
	cd $(WEBOTSNAODIR) && make clean && cd $(CWD)
	cd $(WEBOTSOPDIR) && make clean && cd $(CWD)
	cd $(XOSDIR) && make clean && cd $(CWD)
	cd $(WEBOTSGENERICDIR) && make clean && cd $(CWD)
	cd $(WEBOTSSAFFIRDIR) && make clean && cd $(CWD) 	
	cd $(WEBOTSTHOROPDIR) && make clean && cd $(CWD)
	rm -rf $(WEBOTSCONTDIR) 
	cd $(WEBOTSCHARLIDIR) && make clean && cd $(CWD)
	cd $(OPDIR) && make clean && cd $(CWD)
	cd $(OCCMAPDIR) && make clean && cd $(CWD)
	cd $(PLAYERDIR)/Config && rm -f Config.lua && cd $(CWD)
	cd $(PLAYERDIR) && rm -rf Lib && cd $(CWD)
