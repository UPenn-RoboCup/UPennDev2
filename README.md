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
generalized and versatile robot software framework.


This is a work of the University of Pennsylvania along with help from:
  * IPRE faculty and students: http://calicoproject.org/Main_Page
  * RoMeLa at Virginia Tech: http://www.romela.org/main/Robotics_and_Mechanisms_Laboratory


Copyright:
  All code sources associated with this project are (c) 2013 Stephen McGill, Seung-Joon Yi, Daniel D. Lee, Yida Zhang, Spyridon Karachalios, Qin He.  Exceptions are noted on a per file basis.

Contact Information:
  UPenn E-mail:      upennalizers@gmail.com
  UPenn Website:    https://fling.seas.upenn.edu/~robocup/wiki/index.php
  
`brew install lua boost png libusb jpeg-turbo msgpack zeromq`
`brew link --force jpeg-turbo`
`brew tap smcgill3/grasp-brew`
`brwe install torch openni octomap`
