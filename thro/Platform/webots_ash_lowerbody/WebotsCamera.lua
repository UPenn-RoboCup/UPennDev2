module(..., package.seeall);
require('webots');
require('carray');
require('ImageProc');

webots.wb_robot_init();
timeStep = webots.wb_robot_get_basic_time_step();

-- Get webots tags:
tags = {};
tags.camera = webots.wb_robot_get_device("Camera");

webots.wb_camera_enable(tags.camera, timeStep);
webots.wb_robot_step(timeStep);
height = webots.wb_camera_get_height(tags.camera);
width = webots.wb_camera_get_width(tags.camera);

image = carray.cast(webots.wb_camera_get_image(tags.camera), 'c', 3*height*width);

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

function get_image()
  --rgb2yuyv
  return ImageProc.rgb_to_yuyv(carray.pointer(image), width, height);
end
