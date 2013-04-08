THOR
====

Tactical Hazardous Operations Robot
-----------------------------------

This is the development repository for team THOR.

### For Ubuntu 12.04 and 12.10

#### Install necessary dependencies:

    sudo apt-get install build-essential lua5.1 liblua5.1-0-dev luajit swig libboost1.46-dev mesa-common-dev gnuplot libpopt-dev libncurses5-dev luarocks libblas-dev liblapack-dev libfftw3-dev libhdf5-serial-dev libglfw-dev cmake libmsgpack-dev libtbb-dev

    sudo luarocks install numlua
    sudo luarocks install lua-cmsgpack

    sudo ln -s /usr/bin/luajit* /usr/bin/luajit

#### Install zeromq 3.2:

    wget http://download.zeromq.org/zeromq-3.2.2.tar.gz
    tar xzf zeromq-3.2.2.tar.gz 
    cd zeromq-3.2.2
    ./configure --with-pgm
    make
    sudo make install

    sudo luarocks install https://raw.github.com/Neopallium/lua-zmq/master/rockspecs/lua-zmq-scm-1.rockspec
    sudo ldconfig

#### Install Eigen3:

    wget http://bitbucket.org/eigen/eigen/get/3.1.2.tar.gz
    tar xzf 3.1.2.tar.gz
    cd eigen-eigen-5097c01bcdc4/
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig

#### Install Gazebo:

Ubuntu 12.04:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu precise main" > /etc/apt/sources.list.d/gazebo-latest.list'
Ubuntu 12.10:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu quantal main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu quantal main" > /etc/apt/sources.list.d/gazebo-latest.list'
Both:

    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install gazebo
    echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

#### Install KDL:
Install Kinematics and Dynamics Library (KDL) using:

    git clone http://git.mech.kuleuven.be/robotics/orocos_kinematics_dynamics.git 
    cd orocos_kinematics_dynamics/orocos_kdl
    mkdir build
    cd build
    cmake ..
    make
    sudo make install #(sudo may not be necessary)
    sudo ldconfig

#### Install Torch:

If you would like to test the cognition code, please install torch.

    git clone https://github.com/smcgill3/torch.git
    cd torch
    make
    sudo make install

#### Install OctoMap:

If you would like to test the cognition code, please install OctoMap
from http://octomap.github.io/

### For Mac OSX 10.8:

#### Install necessary dependencies:

- Xcode from the App Store
- XQuartz from http://xquartz.macosforge.org/
- Webots from http://www.cyberbotics.com/
- Homebrew from http://mxcl.github.com/homebrew/

		brew install lua luajit luarocks boost gnuplot eigen swig fftw zmq hdf5 glib wget
		luarocks install numlua lua-cmsgpack
		luarocks install https://raw.github.com/Neopallium/lua-zmq/master/rockspecs/lua-zmq-scm-1.rockspec

#### Install KDL and Torch using the instructions above.  Additionally:

For KDL, use:
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/Cellar/kdl/1.0
make
make install
brew link kdl

Build Instructions
------------------

To build the codebase, run 'make' with one of the following options:

    make ash                  # for operating the physical ASH robot
    make gazebo_ash_lowerbody # for running ASH simulations in gazebo
    make webots_ash           # for running ASH simulations in webots
    make teststand            # for operating the linear actuator teststand
    make robotis_arm          # for operating the robotis arm
    make arm_teststand        # for operating the two armed teststand 
    make tools                # for run-time monitoring using Matlab
