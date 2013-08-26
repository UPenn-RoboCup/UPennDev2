module(..., package.seeall);

require('unix')
Config = require('ConfigPenn')
Transform = require('TransformPenn')
Body = require(Config.Body);


rotating_direction = 1;
body_pos = {0,0,0};
body_rpy = {0,0,0};


function calculate_arm_position()
   trHandle = Transform.eye()
       * Transform.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * Transform.rotZ(handle_yaw)
       * Transform.rotY(handle_pitch);

   trGripL = trHandle
       * Transform.rotX(turnAngle)
       * Transform.trans(0,handle_radius,0)
       * Transform.rotZ(-math.pi/4);
   trGripR = trHandle
       * Transform.rotX(turnAngle)
       * Transform.trans(0,-handle_radius,0)
       * Transform.rotZ(math.pi/4);
       
   trBody=Transform.eye()
       * Transform.trans(body_pos[1],body_pos[2],body_pos[3])
       * Transform.rotZ(body_rpy[3])
		   * Transform.rotY(body_rpy[2]);
		   
   trLArm = Transform.position6D(Transform.inv(trBody)*trGripL);
   trRArm = Transform.position6D(Transform.inv(trBody)*trGripR);
end

function entry()
  print(_NAME..' Entry' ) 
  Body.enable_larm_linear_movement(false); 
  
  phase = 1;
  t0 = unix.time();
  		  --Let's store wheel data here
  handle_pos =  hcm:get_wheel_pos();
  handle_pitch = hcm:get_wheel_pitchangle()[1];
  handle_yaw = hcm:get_wheel_yawangle()[1];
  handle_radius0 = hcm:get_wheel_radius()[1] + 0.08;
  handle_radius1 = hcm:get_wheel_radius()[1];
  turnAngle = 0;

--[[
  print("hpose:",unpack(handle_pos))
  print("hpitch:",handle_pitch)
  print("hradius",handle_radius)
  print("tAngle",turnAngle)
--]]
  
  handle_radius = handle_radius0;
end

t_init = 5.0;
t_grip = 5.0;

function update()
  calculate_arm_position();
  qLArm = Body.get_larm_command_position();
  qRArm = Body.get_rarm_command_position();
  qLInv = Kinematics.inverse_l_arm(trLArm, qLArm);
  qRInv = Kinematics.inverse_r_arm(trRArm, qRArm);

  if phase==1 then
    Body.set_larm_target_position(qLInv); 
    Body.set_rarm_target_position(qRInv); 
    if Body.larm_joint_movement_done() and Body.rarm_joint_movement_done() then
      phase = phase + 1;
      t0 = unix.time();
      return;
    end
  elseif phase==2 then
    Body.set_larm_target_position(qLInv); 
    Body.set_rarm_target_position(qRInv); 
    t = unix.time();
    ph = (t-t0)/t_grip;
    if ph>1 then
      handle_radius = handle_radius1;
      phase = 3;
      Body.set_lhand_position(Config.arm.FingerClosed);
      Body.set_rhand_position(Config.arm.FingerClosed);
      return "done";  
    end
    handle_radius = handle_radius0*(1-ph) + ph*handle_radius1;
  end
end

function exit()
  print(_NAME..' Exit' ) 
end
 
