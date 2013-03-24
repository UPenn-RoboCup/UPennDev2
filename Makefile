# Define environmental variables
###########################################################################
CWD = $(shell pwd)
include Makefile.inc

# Define targets
###########################################################################

all:
	@echo " Please choose one of the following targets:"
	@echo " make ash"
	@echo " make gazebo_ash_lowerbody"
	@echo " make webots_ash"
	@echo " make teststand"
	@echo " make arm_teststand"
	@echo " make robotis_arm"
	@echo " make tools"
	@echo " make clean"

ash robotis_arm arm_teststand teststand:
	cd Framework/Lib && make && cd $(CWD)
	cd Platform/Lib && make && cd $(CWD)
	cd Platform/$@ && make && cd $(CWD)
	cd Framework/Platform \
	&& rm -f ./* \
	&& ln -s ../../Platform/$@/Platform.lua Platform.lua \
	&& ln -s ../../Platform/$@/Mechanics/Kinematics.$(SHLIBEXT) Kinematics.$(SHLIBEXT) \
	&& ln -s ../../Platform/$@/Mechanics/Dynamics.$(SHLIBEXT) Dynamics.$(SHLIBEXT) \
	&& cd $(CWD)
	cd Config \
	&& rm -f Config*.lua \
	&& ln -s $@/* .\
	&& cd $(CWD)
	cd Run \
	&& rm -f init_robot \
	&& rm -f comms_manager \
	&& ln -s ../Platform/$@/Init/init_robot init_robot \
	&& ln -s ../Platform/$@/Comms/comms_manager comms_manager \
	&& cd $(CWD)

gazebo_ash_lowerbody :
	cd Framework/Lib && make && cd $(CWD)
	cd Platform/Lib/KDL && make && cd $(CWD)
	cd Platform/$@ && make && cd $(CWD)
	cd Framework/Platform \
	&& rm -f ./* \
	&& ln -s ../../Platform/$@/Platform.lua Platform.lua \
	&& ln -s ../../Platform/$@/Sensor.lua Sensor.lua \
	&& ln -s ../../Platform/$@/Mechanics/Kinematics.$(SHLIBEXT) Kinematics.$(SHLIBEXT) \
	&& ln -s ../../Platform/$@/Mechanics/Dynamics.$(SHLIBEXT) Dynamics.$(SHLIBEXT) \
	&& cd $(CWD)
	cd Config \
	&& rm -f Config*.lua \
	&& ln -s $@/* .\
	&& cd $(CWD)
	cd Run \
	&& rm -f init_robot \
	&& rm -f comms_manager \
	&& ln -s ../Platform/$@/Comms/comms_manager comms_manager \
	&& cd $(CWD)

webots_ash webots_ash_lowerbody :
	cd Framework/Lib && make && cd $(CWD)
	cd Framework/Lib/webots && make && cd $(CWD)
	cd Platform/Lib/KDL && make && cd $(CWD)
	cd Platform/$@ && make && cd $(CWD)
	cd Framework/Platform \
	&& rm -f ./* \
	&& ln -s ../../Platform/$@/Platform.lua Platform.lua \
	&& ln -s ../../Platform/$@/Sensor.lua Sensor.lua \
	&& ln -s ../../Platform/$@/Mechanics/Kinematics.$(SHLIBEXT) Kinematics.$(SHLIBEXT) \
	&& ln -s ../../Platform/$@/Mechanics/Dynamics.$(SHLIBEXT) Dynamics.$(SHLIBEXT) \
	&& cd $(CWD)
	cd Config \
	&& rm -f Config*.lua \
	&& ln -s $@/* .\
	&& cd $(CWD)
	rm -f Webots \
	&& ln -s Platform/$@/Project Webots
	cd Run \
	&& rm -f init_robot \
	&& rm -f comms_manager \
	&& cd $(CWD)

vrep_ash:
	cd Framework/Lib && make && cd $(CWD)
	cd Platform/Lib/KDL && make && cd $(CWD)
	cd Platform/$@/Mechanics && make && cd $(CWD)
	cd Framework/Platform \
	&& rm -f ./* \
	&& ln -s ../../Platform/$@/Platform.lua Platform.lua \
	&& ln -s ../../Platform/$@/Mechanics/Kinematics.$(SHLIBEXT) Kinematics.$(SHLIBEXT) \
	&& ln -s ../../Platform/$@/Mechanics/Dynamics.$(SHLIBEXT) Dynamics.$(SHLIBEXT) \
	&& ln -s ../../Platform/$@/Comms/vrep_impedance_controller.lua vrep_impedance_controller.lua \
	&& ln -s ../../Platform/$@/Comms/vrep_position_controller.lua vrep_position_controller.lua \
	&& ln -s ../../Platform/$@/Comms/vrep_velocity_controller.lua vrep_velocity_controller.lua \
	&& ln -s ../../Platform/$@/Comms/vrep_comms_manager.lua vrep_comms_manager.lua \
	&& cd $(CWD)
	cd Config \
	&& rm -f Config*.lua \
	&& ln -s $@/* .\
	&& cd $(CWD)
	cd Run \
	&& rm -f init_robot \
	&& rm -f comms_manager \
	&& ln -s ../Platform/$@/Comms/comms_manager comms_manager \
	&& cd $(CWD)

sensors:
	cd Framework/Lib && make && cd $(CWD)
	cd Config \
	&& rm -f Config*.lua \
	&& ln -s $@/* .\
	&& cd $(CWD)

tools:
	cd Tools/Lib && make && cd $(CWD)

clean:
	rm -f Framework/Platform/*
	rm -f Config/Config*.lua
	rm -f Webots
	rm -f Run/init_robot
	rm -f Run/comms_manager
	cd Framework/Lib && make clean && cd $(CWD)
	cd Framework/Lib/webots && make clean && cd $(CWD)
	cd Platform/Lib && make clean && cd $(CWD)
	cd Tools/Lib && make clean && cd $(CWD)
	cd Platform/ash && make clean && cd $(CWD)
	cd Platform/teststand && make clean && cd $(CWD)
	cd Platform/robotis_arm && make clean && cd $(CWD)
	cd Platform/arm_teststand && make clean && cd $(CWD)
	cd Platform/vrep_ash && make clean && cd $(CWD)
	cd Platform/webots_ash && make clean && cd $(CWD)
	cd Platform/webots_ash_lowerbody && make clean && cd $(CWD)
	cd Platform/gazebo_ash_lowerbody && make clean && cd $(CWD)

.PHONY: all ash teststand arm_teststand robotis_arm webots_ash webots_ash_lowerbody vrep_ash tools clean
