require('Camera');
require('Detection');

if (Config.camera.width ~= Camera.get_width()
    or Config.camera.height ~= Camera.get_height()) then
  print('Camera width/height mismatch');
  print('Config width/height = ('..Config.camera.width..', '..Config.camera.height..')');
  print('Camera width/height = ('..Camera.get_width()..', '..Camera.get_height()..')');
  error('Config file is not set correctly for this camera. Ensure the camera width and height are correct.');
end
vcm.set_image_width(Config.camera.width);
vcm.set_image_height(Config.camera.height);

camera = {};

camera.width = Camera.get_width();
camera.height = Camera.get_height();
camera.npixel = camera.width*camera.height;
camera.image = Camera.get_image();
camera.status = Camera.get_camera_status();
camera.switchFreq = Config.camera.switchFreq;
camera.ncamera = Config.camera.ncamera;
-- Initialize the Labeling
labelA = {};
-- labeled image is 1/4 the size of the original
labelA.m = camera.width/2;
labelA.n = camera.height/2;
labelA.npixel = labelA.m*labelA.n;
if  webots == 1 then
  labelA.m = camera.width;
  labelA.n = camera.height;
  labelA.npixel = labelA.m*labelA.n;
end
scaleB = Config.vision.scaleB;
labelB = {};
labelB.m = labelA.m/scaleB;
labelB.n = labelA.n/scaleB;
labelB.npixel = labelB.m*labelB.n;
vcm.set_image_scaleB(Config.vision.scaleB);
print('Vision LabelA size: ('..labelA.m..', '..labelA.n..')');
print('Vision LabelB size: ('..labelB.m..', '..labelB.n..')');

function update_gps_only()
  --We are now using ground truth robot and ball pose data
  headAngles = Body.get_head_position();
  --TODO: camera select
--  HeadTransform.update(status.select, headAngles);
  HeadTransform.update(0, headAngles);
  
  --update FOV
  update_shm_fov()

  --Get GPS coordinate of robot and ball
  gps_pose = wcm.get_robot_gpspose();
  ballGlobal=wcm.get_robot_gps_ball();  
  
  --Check whether ball is inside FOV
  ballLocal = util.pose_relative(ballGlobal,gps_pose);
 
  --Get the coordinates of FOV boundary
  local v_TL = vcm.get_image_fovTL();
  local v_TR = vcm.get_image_fovTR();
  local v_BL = vcm.get_image_fovBL();
  local v_BR = vcm.get_image_fovBR();

--[[
print("BallLocal:",unpack(ballLocal))
print("V_TL:",unpack(v_TL))
print("V_TR:",unpack(v_TR))
print("V_BL:",unpack(v_BL))
print("V_BR:",unpack(v_BR))
print("Check 1:",
   check_side(v_TL, ballLocal, v_TR));
print("Check 2:",
     check_side(v_TL, v_BL, ballLocal) );
print("Check 3:",
     check_side(v_BR, v_TR, ballLocal) );
print("Check 4:",
     check_side(v_BL, v_BR, ballLocal) );
--]]

  --Check whether ball is within FOV boundary 
  if check_side(v_TR, v_TL, ballLocal) < 0 and
     check_side(v_TL, v_BL, ballLocal) < 0 and
     check_side(v_BR, v_TR, ballLocal) < 0 and
     check_side(v_BL, v_BR, ballLocal) < 0 then
    vcm.set_ball_detect(1);
  else
    vcm.set_ball_detect(0);
  end


end

