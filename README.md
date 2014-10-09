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
