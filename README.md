This project is a modularized software framework for use with humanoid robot
development and research. The modularized platform separates low level
components that vary from robot to robot from the high level logic that does not
vary across robots. The low level components include processes to communicate
with motors and sensors on the robot, including the camera. The high level
components include the state machines that control how the humanoids move around
and process sensor data. By separating into these levels, we achieve a more
adaptable system that is easily ported to different humanoids.

The project began with the University of Pennsylvania RoboCup code base from
the 2011 RoboCup season and is continuing to evolve into an ever more
generalized and versatile robot software framework.  The DARPA Robotics Challenge also pushed development

Copyright:
  All code sources associated with this project are (c) 2013 Stephen McGill, Seung-Joon Yi, Daniel D. Lee, Yida Zhang, Spyridon Karachalios, Qin He, Jordan Brindza.  Exceptions are noted on a per file basis.

Contact Information:
  UPenn E-mail:     upennalizers@gmail.com
  UPenn Website:    https://fling.seas.upenn.edu/~robocup/wiki/index.php
  
Mac
* `brew install lua boost png libusb jpeg-turbo msgpack zeromq swig`
* `brew link --force jpeg-turbo`

Ubuntu
* Install dependencies https://github.com/smcgill3/UPennDev/wiki/install-dependencies-for-UpennDev-in-Ubuntu

Torch
* git clone https://github.com/smcgill3/torch7.git
* cd torch7
* make prep
* make -j4

Making
* make clean
* make -j8
* make THOROP

speaker-test -c1 -Dsysdefault:Device
pulseaudio -k; and sudo alsa force-reload

sudo chown -R thor /usr/local
sudo usermod -a -G dialout thor
sudo apt-get install git htop build-essential gfortran automake libudev-dev pkg-config zlib1g-dev libpcre3-dev liblzma-dev libreadline-dev libpng12-dev libjpeg-dev

cd ~/
mkdir -p src
cd src

git clone http://luajit.org/git/luajit-2.0.git
cd luajit-2.0
git checkout v2.1
make
make install
ln -sf luajit-2.1.0-alpha /usr/local/bin/luajit
cd ~/src

wget http://downloads.sourceforge.net/project/boost/boost/1.57.0/boost_1_57_0.tar.bz2
tar xvvf boost_1_57_0.tar.bz2
cd /usr/local
sudo ln -s ~/src/boost_1_57_0/boost .

git clone https://github.com/libusb/libusb.git
cd libusb
** APPLY https://github.com/OpenKinect/libfreenect2/blob/master/depends/linux_usbfs_increase_max_iso_buffer_length.patch **
./autogen.sh
make
make install

git clone https://github.com/msgpack/msgpack-c.git
cd msgpack-c
./bootstrap
./configure
make
make install

wget http://download.zeromq.org/zeromq-4.0.5.tar.gz
tar xvvf zeromq-4.0.5.tar.gz
cd zeromq-4.0.5
./autogen.sh
./configure
make
make install PREFIX=/usr/local

git clone https://github.com/xianyi/OpenBLAS.git
cd OpenBLAS
make
make install

git clone https://github.com/smcgill3/torch7.git
cd torch7
git checkout build-fixes
make prep
make
make install
cd ~/src

git clone git clone https://github.com/ggreer/the_silver_searcher.git
cd the_silver_searcher
./build.sh
make install

# Fish
sudo apt-get install build-essential ncurses-dev libncurses5-dev gettext

cd ~/
git clone https://github.com/smcgill3/UPennDev.git

  GNU nano 2.2.6                       File: /etc/udev/rules.d/55-kinect2-usb.rules

# Make kinect2 device mount with writing permissions (default is read only for unknown devices)
# https://github.com/PrimeSense/Sensor/blob/unstable/Platform/Linux/Install/55-primesense-usb.rules
SUBSYSTEM=="usb", ATTR{idProduct}=="02d8", ATTR{idVendor}=="045e", MODE:="0666", OWNER:="thor", GROUP:="video"
SUBSYSTEM=="usb", ATTR{idProduct}=="02d9", ATTR{idVendor}=="045e", MODE:="0666", OWNER:="thor", GROUP:="video"

sudo ldconfig
