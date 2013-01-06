module(..., package.seeall);
require('controller');
require('carray');
require('ImageProc');
require('unix')
require('Body')
require('vcm')



positionTop = 0;
positionBottom = 0.70;

height =vcm.get_image_height();
width = vcm.get_image_width();

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

function get_image()
  mycount = mycount + 1;
  return vcm.get_image_yuyv();
end

function get_labelA( lut )
  return vcm.get_image_labelA();
end

function get_camera_status()
  status = {};
  status.select = 0;
  status.count = mycount;
  status.time = unix.time();
  status.joint = vector.zeros(20);
  mycount = mycount + 1;
  return status;
end

function get_camera_position()
  return 0;
end

function select_camera()
end

function get_select()
  return 0;
end

