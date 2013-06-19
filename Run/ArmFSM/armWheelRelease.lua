module(..., package.seeall);

require('unix')
Config = require('ConfigPenn')
Transform = require('TransformPenn')

Body = require(Config.Body);

handle_pos = {0.30,0,0.10};
handle_pitch = 0;
handle_radius = 0.20;
turnAngle = 0;

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
  Body.set_lhand_position(0.1);
  Body.set_rhand_position(0.1);
  phase = 1;
  t0 = unix.time();
  
  handle_pos =  hcm:get_wheel_pos();
  handle_pitch = hcm:get_wheel_pitchangle()[1];
  handle_yaw = hcm:get_wheel_yawangle()[1];
  handle_radius0 = hcm:get_wheel_radius()[1];
  handle_radius1 = hcm:get_wheel_radius()[1]+0.08;
  handle_turnangle0 = hcm:get_wheel_turnangle()[1];    

  Body.set_lhand_position(Config.arm.FingerOpen);
  Body.set_rhand_position(Config.arm.FingerOpen); 
end

t_return = 2.0;
t_release = 3.0;

function update()
  t = unix.time();
  if phase==1 then
    ph = (t-t0)/t_return; 
    if ph>1 then
      phase = 2;
      t0 = unix.time();
      return;
    end  
    turnAngle = (1-ph) * handle_turnangle0; 
    handle_radius = handle_radius0;
  elseif phase==2 then
    ph = (t-t0)/t_release;
    if ph>1 then
      return "done";  
    end
    handle_radius = handle_radius0*(1-ph) + ph*handle_radius1;
  end
  calculate_arm_position();
  
  qLArm = Body.get_larm_command_position();
  qRArm = Body.get_rarm_command_position(); 
  qLInv = Kinematics.inverse_l_arm(trLArm, qLArm);
  qRInv = Kinematics.inverse_r_arm(trRArm, qRArm);
  Body.set_larm_target_position(qLInv); 
  Body.set_rarm_target_position(qRInv); 
  
end

function exit()
 --super slow
  dArmVelAngle = vector.new({10,10,10,15,45,45})*math.pi/180; --30 degree per second
  Body.set_arm_movement_velocity(dArmVelAngle);

  print(_NAME..' Exit' ) 
end
