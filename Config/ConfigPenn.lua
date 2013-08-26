module(..., package.seeall);
require('vector')

------------------------------------------------
-- Main config file for Penn codes
-- We define udp ip addresses and ports here
------------------------------------------------

RAD = math.pi/180;

udp={}
udp.UDP_IP = '192.168.123.255'; --for actual udp
--udp.UDP_IP = '127.0.0.1'; --for testing

udp.PORT_CONTROL = 54344;  	--for human control
udp.PORT_LIDAR = 54345;    	--for lidar data
udp.PORT_FEEDBACK = 54329; 	--for joint angles and etc. feedback
udp.PORT_HCAMERA = 54321;   --for camera message
udp.PORT_LCAMERA = 54322;   --for camera message
udp.PORT_RCAMERA = 54323;   --for camera message
udp.PORT_SLAMMAP = 43210;  	--for slam map
udp.PORT_COMMAND = 43211;  	--for command from HRI
udp.PORT_HEIGHTMAP = 43230; --for height map

Body = 'ThorCentaurBody';

use_odometry_only = false  --true;
use_odometry_only = true;

movement={};
movement.maxFPS = 50;
movement.lidar0={}
movement.lidar1={}

movement.lidar0.pan_active = false;
movement.lidar0.angle_step = 0.25*RAD;
movement.lidar0.angle0 = 10*RAD;
movement.lidar0.angle1 = 70*RAD;

movement.lidar1.pan_active = true;
movement.lidar1.angle_step = 1*RAD;
movement.lidar1.angle0 = -60*RAD;
movement.lidar1.angle1 = 60*RAD;


slam = {}
slam.jpeg_quality = 60


camera={}
camera.use_wrist_camera = false;
--camera.head_device = '/dev/headcam'
--camera.left_device = '/dev/leftcam'
--camera.right_device = '/dev/rightcam'
--camera.head_device = '/dev/headcam'
--camera.head_device = '/dev/video0'
camera.head_device = '/dev/headcam'


camera.left_device = '/dev/video1'
camera.right_device = '/dev/video2'

camera.head_quality = 50;
--camera.head_resolution = {800,448};
camera.head_resolution = {640,360};
--camera.head_resolution = {432,240};

camera.head_fps = 30;
camera.hand_quality = 50;
camera.hand_resolution = {320,240};
camera.hand_fps = 30;

dcm = {}
dcm.targetFPS = 50;
dcm.read_enable = 1;
dcm.read_enable = 0;

lidar = {}
lidar.head_serial = "00907258"
lidar.chest_serial = "00805676"
lidar.depth_quality = 90
lidar.use_second_lidar = true
lidar.lidar_0_head = true

lidar.lidar0={}
lidar.lidar0.range0 = -180; -- -45 deg
lidar.lidar0.range1 = 180; -- 45 deg
lidar.lidar0.lidarrange = 5.12; --2cm resolution
--lidar.lidar0.lidarrange = 1.28; 

lidar.lidar1={}
lidar.lidar1.range0 = -120; -- -30 deg
lidar.lidar1.range1 = 240; -- 60 deg
lidar.lidar1.lidarrange = 5.0; 


arm={}
arm.qLArmInit={
    vector.new({90,0,90,-0,-90,0})*RAD,
    vector.new({90,90,90,-90,-90,0})*RAD,
    vector.new({0,90,90,-90,-90,-45})*RAD,
  }

arm.qRArmInit={
    vector.new({90,-0,-90,-0,90,0})*RAD,
    vector.new({90,-90,-90,-90,90,0})*RAD,
    vector.new({0,-90,-90,-90,90,45})*RAD,
  }


arm.FingerClosed = 0.9;
arm.FingerOpen = 0.1;
