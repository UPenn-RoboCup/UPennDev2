# Define environmental variables
###########################################################################
CWD = $(shell pwd)
include Makefile.inc

# Define targets
###########################################################################

all:
	@echo " Please choose one of the following targets:"
	@echo " make ash"
	@echo " make vrep_ash"
	@echo " make webots_ash"
	@echo " make teststand"
	@echo " make robotis_arm"
	@echo " make tools"
	@echo " make clean"

ash:
	cd Framework/Lib && make && cd $(CWD)
	cd Platform/Lib && make && cd $(CWD)
	cd Platform/ASH && make && cd $(CWD)
	cd Framework/Platform \
	&& rm -f ./* \
	&& ln -s ../../Platform/ASH/Platform.lua Platform.lua \
	&& ln -s ../../Platform/ASH/Mechanics/Kinematics.$(SHLIBEXT) Kinematics.$(SHLIBEXT) \
	&& ln -s ../../Platform/ASH/Mechanics/Dynamics.$(SHLIBEXT) Dynamics.$(SHLIBEXT) \
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
	cd Framework/Platform \
	&& rm -f ./* \
	&& ln -s ../../Platform/Teststand/Platform.lua Platform.lua \
	&& ln -s ../../Platform/Teststand/Mechanics/Kinematics.lua Kinematics.lua \
	&& ln -s ../../Platform/Teststand/Mechanics/Dynamics.lua Dynamics.lua \
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

vrep_ash:
	cd Framework/Lib && make && cd $(CWD)
	cd Platform/Lib/KDL && make && cd $(CWD)
	cd Platform/VRepASH/Mechanics && make && cd $(CWD)
	cd Framework/Platform \
	&& rm -f ./* \
	&& ln -s ../../Platform/VRepASH/Platform.lua Platform.lua \
	&& ln -s ../../Platform/VRepASH/VRepCommsManager.lua VRepCommsManager.lua \
	&& ln -s ../../Platform/VRepASH/Mechanics/Kinematics.$(SHLIBEXT) Kinematics.$(SHLIBEXT) \
	&& ln -s ../../Platform/VRepASH/Mechanics/Dynamics.$(SHLIBEXT) Dynamics.$(SHLIBEXT) \
	&& cd $(CWD)
	cd Config \
	&& rm -f Config.lua \
	&& ln -s Config_ASH.lua Config.lua \
	&& cd $(CWD)
	cd Run \
	&& rm -f init_robot \
	&& rm -f comms_manager \
	&& cd $(CWD)

webots_ash:
	cd Framework/Lib && make && cd $(CWD)
	cd Framework/Lib/webots && make && cd $(CWD)
	cd Platform/Lib/KDL && make && cd $(CWD)
	cd Platform/WebotsASH && make && cd $(CWD)
	cd Framework/Platform \
	&& rm -f ./* \
	&& ln -s ../../Platform/WebotsASH/Platform.lua Platform.lua \
	&& ln -s ../../Platform/WebotsASH/Sensor.lua Sensor.lua \
	&& ln -s ../../Platform/WebotsASH/Mechanics/Kinematics.$(SHLIBEXT) Kinematics.$(SHLIBEXT) \
	&& ln -s ../../Platform/WebotsASH/Mechanics/Dynamics.$(SHLIBEXT) Dynamics.$(SHLIBEXT) \
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

webots_ash_lowerbody:
	cd Framework/Lib && make && cd $(CWD)
	cd Framework/Lib/webots && make && cd $(CWD)
	cd Platform/Lib/KDL && make && cd $(CWD)
	cd Platform/WebotsASHLowerbody && make && cd $(CWD)
	cd Framework/Platform \
	&& rm -f ./* \
	&& ln -s ../../Platform/WebotsASHLowerbody/Platform.lua Platform.lua \
	&& ln -s ../../Platform/WebotsASHLowerbody/Sensor.lua Sensor.lua \
	&& ln -s ../../Platform/WebotsASHLowerbody/Mechanics/Kinematics.$(SHLIBEXT) Kinematics.$(SHLIBEXT) \
	&& ln -s ../../Platform/WebotsASHLowerbody/Mechanics/Dynamics.$(SHLIBEXT) Dynamics.$(SHLIBEXT) \
	&& cd $(CWD)
	cd Config \
	&& rm -f Config.lua \
	&& ln -s Config_WebotsASHLowerbody.lua Config.lua \
	&& cd $(CWD)
	rm -f Webots \
	&& ln -s Platform/WebotsASHLowerbody/Project Webots
	cd Run \
	&& rm -f init_robot \
	&& rm -f comms_manager \
	&& cd $(CWD)

robotis_arm:
	cd Framework/Lib && make && cd $(CWD)
	cd Platform/Lib && make && cd $(CWD)
	cd Platform/RobotisArm && make && cd $(CWD)
	cd Framework/Platform \
	&& rm -f ./* \
	&& ln -s ../../Platform/RobotisArm/Platform.lua Platform.lua \
	&& ln -s ../../Platform/RobotisArm/Mechanics/Kinematics.so Kinematics.so \
	&& ln -s ../../Platform/RobotisArm/Mechanics/Dynamics.lua Dynamics.lua \
	&& cd $(CWD)
	cd Config \
	&& rm -f Config.lua \
	&& ln -s Config_RobotisArm.lua Config.lua \
	&& cd $(CWD)
	cd Run \
	&& rm -f init_robot \
	&& rm -f comms_manager \
	&& ln -s ../Platform/RobotisArm/Init/init_robot init_robot \
	&& ln -s ../Platform/RobotisArm/Comms/comms_manager comms_manager \
	&& cd $(CWD)

tools:
	cd Tools/Lib && make && cd $(CWD)

clean:
	rm -f Framework/Platform/*
	rm -f Config/Config.lua
	rm -f Webots
	rm -f Run/init_robot
	rm -f Run/comms_manager
	cd Framework/Lib && make clean && cd $(CWD)
	cd Framework/Lib/webots && make clean && cd $(CWD)
	cd Platform/Lib && make clean && cd $(CWD)
	cd Tools/Lib && make clean && cd $(CWD)
	cd Platform/ASH && make clean && cd $(CWD)
	cd Platform/Teststand && make clean && cd $(CWD)
	cd Platform/WebotsASH && make clean && cd $(CWD)
	cd Platform/RobotisArm && make clean && cd $(CWD)

.PHONY: all ash teststand webots_ash robotis_arm tools clean
