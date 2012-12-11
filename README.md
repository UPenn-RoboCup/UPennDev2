THOR
====

Tactical Hazardous Operations Robot
-----------------------------------

This is the development repository for team THOR.

Dependencies
------------

For Ubuntu 12.04, install necessary dependencies using:

    sudo apt-get install build-essential lua5.1 liblua5.1-0-dev luajit swig \
    libboost1.46-dev mesa-common-dev gnuplot libpopt-dev libncurses5-dev

    sudo ln -s /usr/bin/luajit* /usr/bin/luajit

Install the numlua library using:
    
    sudo apt-get install luarocks libblas-dev liblapack-dev libfftw3-dev \
    libhdf5-serial-dev

    git clone https://github.com/carvalho/numlua
    cd numlua
    sudo luarocks make numlua-0.3-1.rockspec

Install the Kinematics and Dynamics Library using:

    sudo apt-get install libeigen2-dev

    git clone http://git.mech.kuleuven.be/robotics/orocos_kinematics_dynamics.git 
    cd orocos_kinematics_dynamics/orocos_kdl
    mkdir build
    cd build
    cmake ..
    make
    sudo make install

For Mac OSX 10.8, install necessary dependencies using:

		Xcode from the App Store
		XQuartz from http://xquartz.macosforge.org/
		Webots from http://www.cyberbotics.com/
		Homebrew from http://mxcl.github.com/homebrew/
		
		brew install lua boost gnuplot
		

Build Instructions
------------------

To build the codebase, run 'make' with one of the following options:

    make ash         # for operating the physical ASH robot
    make webots_ash  # for running ASH simulations in webots
    make teststand   # for operating the linear actuator teststand
    make robotis_arm # for operating the robotis arm
    make tools       # for run-time monitoring using Matlab
