THOR
====

Tactical Hazardous Operations Robot
-----------------------------------

This is the development repository for team THOR.

Dependencies
------------

#### For Ubuntu 12.04, install necessary dependencies using:

    sudo apt-get install build-essential lua5.1 liblua5.1-0-dev luajit swig libboost1.46-dev mesa-common-dev gnuplot libpopt-dev libncurses5-dev luarocks libblas-dev liblapack-dev libfftw3-dev libhdf5-serial-dev

    sudo luarocks install numlua lua-cmsgpack

    sudo ln -s /usr/bin/luajit* /usr/bin/luajit

Install Eigen3 using:

    wget http://bitbucket.org/eigen/eigen/get/3.1.2.tar.gz
    tar xzf 3.1.2.tar.gz
    cd eigen-eigen-5097c01bcdc4/
    mkdir build
    cd build
    cmake ..
    make
    sudo make install

Install zeromq 3.2 using:

    wget http://download.zeromq.org/zeromq-3.2.2.tar.gz
    tar xzf zeromq-3.2.2.tar.gz 
    cd zeromq-3.2.2
    ./configure --with-pgm
    make
    sudo make install

    sudo luarocks install https://raw.github.com/Neopallium/lua-zmq/master/rockspecs/lua-zmq-scm-1.rockspec

Finally, install Lightweight Communications and Marshalling (LCM) and Kinematics and Dynamics Library (KDL) using the instructions below.

#### For Mac OSX 10.8, install necessary dependencies using:

- Xcode from the App Store
- XQuartz from http://xquartz.macosforge.org/
- Webots from http://www.cyberbotics.com/
- Homebrew from http://mxcl.github.com/homebrew/

		brew install lua luajit luarocks boost gnuplot eigen swig fftw zmq hdf5 glib wget
		luarocks install numlua lua-cmsgpack
		luarocks install https://raw.github.com/Neopallium/lua-zmq/master/rockspecs/lua-zmq-scm-1.rockspec

Finally, install Lightweight Communications and Marshalling (LCM) and Kinematics and Dynamics Library (KDL) using the instructions below.

#### LCM and KDL
Install Lightweight Communications and Marshalling (LCM) using:

    wget https://lcm.googlecode.com/files/lcm-0.9.2.tar.gz
    tar xzf lcm-0.9.2.tar.gz
    cd lcm-0.9.2
    ./configure
    make
    sudo make install #(sudo may not be necessary)

Install Kinematics and Dynamics Library (KDL) using:

    git clone http://git.mech.kuleuven.be/robotics/orocos_kinematics_dynamics.git 
    cd orocos_kinematics_dynamics/orocos_kdl
    mkdir build
    cd build
    cmake ..
    make
    sudo make install #(sudo may not be necessary)

#### Torch

If you would like to test the ognition code, please install torch.

    sudo apt-get install libblas-dev liblapack-dev

    git clone https://github.com/smcgill3/torch.git
    cd torch
    make
    sudo make install

Build Instructions
------------------

To build the codebase, run 'make' with one of the following options:

    make ash         # for operating the physical ASH robot
    make webots_ash  # for running ASH simulations in webots
    make teststand   # for operating the linear actuator teststand
    make robotis_arm # for operating the robotis arm
    make tools       # for run-time monitoring using Matlab
