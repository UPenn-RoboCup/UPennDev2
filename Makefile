CWD = $(shell pwd)

# Define environmental variables
###########################################################################
ifndef OSTYPE
  export OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
endif

ifeq ($(OSTYPE),darwin)
  export WEBOTS_HOME = /Applications/Webots
else
  export WEBOTS_HOME = /usr/local/webots
endif

# Define targets
###########################################################################

all:
	@echo " Please choose one of the following targets:"
	@echo " make ash"
	@echo " make teststand"
	@echo " make webots_ash"
	@echo " make tools"
	@echo " make clean"

ash:
	cd Framework/Lib && make && cd $(CWD)
	cd Platform/Lib && make && cd $(CWD)
	cd Platform/ASH && make && cd $(CWD)
	cd Framework/Robot \
	&& rm -f Body.lua Kinematics.so Statics.so \
	&& ln -s ../../Platform/ASH/Body.lua Body.lua \
	&& ln -s ../../Platform/ASH/Mechanics/Kinematics.so Kinematics.so \
	&& ln -s ../../Platform/ASH/Mechanics/Statics.so Statics.so \
	&& cd $(CWD)
	cd Config \
	&& rm -f Config.lua \
	&& ln -s Config_ASH.lua Config.lua \
	&& cd $(CWD)
	cd Run \
	&& rm -f init_robot \
	&& rm -f comms_manager \
	&& ln -s ../Platform/ASH/Init/init_robot init_robot \
	&& ln -s ../Platform/ASH/Comms/comms_manager comms_manager \
	&& cd $(CWD)

teststand:
	cd Framework/Lib && make && cd $(CWD)
	cd Platform/Lib && make && cd $(CWD)
	cd Platform/Teststand && make && cd $(CWD)
	cd Framework/Robot \
	&& rm -f Body.lua Kinematics.so Statics.so \
	&& ln -s ../../Platform/Teststand/Body.lua Body.lua \
	&& ln -s ../../Platform/Teststand/Mechanics/Kinematics.so Kinematics.so \
	&& ln -s ../../Platform/Teststand/Mechanics/Statics.so Statics.so \
	&& cd $(CWD)
	cd Config \
	&& rm -f Config.lua \
	&& ln -s Config_Teststand.lua Config.lua \
	&& cd $(CWD)
	cd Run \
	&& rm -f init_robot \
	&& rm -f comms_manager \
	&& ln -s ../Platform/Teststand/Init/init_robot init_robot \
	&& ln -s ../Platform/Teststand/Comms/comms_manager comms_manager \
	&& cd $(CWD)

webots_ash:
	export WEBOTS_HOME=$(WEBOTS_HOME)
	cd Framework/Lib && make && cd $(CWD)
	cd Framework/Lib/webots && make && cd $(CWD)
	cd Platform/WebotsASH && make && cd $(CWD)
	cd Framework/Robot \
	&& rm -f Body.lua Kinematics.so Statics.so \
	&& ln -s ../../Platform/WebotsASH/Body.lua Body.lua \
	&& ln -s ../../Platform/WebotsASH/Mechanics/Kinematics.so Kinematics.so \
	&& ln -s ../../Platform/WebotsASH/Mechanics/Statics.so Statics.so \
	&& cd $(CWD)
	cd Config \
	&& rm -f Config.lua \
	&& ln -s Config_WebotsASH.lua Config.lua \
	&& cd $(CWD)
	rm -f Webots \
	&& ln -s Platform/WebotsASH/Project Webots
	cd Run \
	&& rm -f init_robot \
	&& rm -f comms_manager \
	&& cd $(CWD)

tools:
	cd Tools/Lib && make && cd $(CWD)

clean:
	cd Framework/Lib && make clean && cd $(CWD)
	cd Framework/Lib/webots && make clean && cd $(CWD)
	cd Platform/Lib && make clean && cd $(CWD)
	cd Tools/Lib && make clean && cd $(CWD)
	cd Platform/ASH && make clean && cd $(CWD)
	cd Platform/Teststand && make clean && cd $(CWD)
	cd Platform/WebotsASH && make clean && cd $(CWD)
	rm -f Framework/Robot/Body.lua
	rm -f Framework/Robot/Kinematics.so
	rm -f Framework/Robot/Statics.so
	rm -f Config/Config.lua
	rm -f Webots
	rm -f Run/init_robot
	rm -f Run/comms_manager

.PHONY: all ash teststand webots_ash tools clean
