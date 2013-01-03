module(..., package.seeall);
require('webots')
require('carray');
require('unix')
require('Body')

webots.wb_robot_init();
timeStep = webots.wb_robot_get_basic_time_step();

-- Get webots tags:
tags = {};
tags.laser = webots.wb_robot_get_device("UTM-30LX");

webots.wb_camera_enable(tags.laser, timeStep);
webots.wb_robot_step(timeStep);
height = webots.wb_camera_get_height(tags.laser);
width = webots.wb_camera_get_width(tags.laser);
-- 1 'f' (float) per laser return
image = carray.cast(webots.wb_camera_get_image(tags.laser),'f', height*width*1);
mycount = 0;

function set_param()
end

function get_param()
  return 0;
end

function get_height()
  return height;
end

function get_width()
  return width;
end

function get_scan()
	return image;
  --rgb2yuyv
  --return ImageProc.rgb_to_yuyv(carray.pointer(image), width, height);
end

function get_camera_status()
  status = {};
  status.select = 0;
  status.count = mycount;
  status.time = unix.time();
  status.joint = vector.zeros(20);
  tmp = Body.get_head_position();
  status.joint[1],status.joint[2] = tmp[1], tmp[2];
  mycount = mycount + 1;
  return status;
end

function get_camera_position()
  return 0;
--  return webots.wb_servo_get_position(tags.laserSelect);
end

function select_camera()
end

function get_select()
  return 0;
end

