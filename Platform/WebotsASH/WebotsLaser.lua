module(..., package.seeall);
require('webots')
require('carray');


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
end
