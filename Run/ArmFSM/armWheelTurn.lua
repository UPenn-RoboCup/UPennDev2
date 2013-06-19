module(..., package.seeall);

require('unix')
Config = require('ConfigPenn')
Transform = require('TransformPenn')

Body = require(Config.Body);

handle_pos = {0.30,0,0.10};
handle_pitch = 0;
handle_radius = 0.15;
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
  t0 = unix.time();  
  
  handle_pos =  hcm:get_wheel_pos();
  handle_pitch = hcm:get_wheel_pitchangle()[1];
  handle_yaw = hcm:get_wheel_yawangle()[1];
  handle_radius = hcm:get_wheel_radius()[1];
  turnAngle = hcm:get_wheel_turnangle()[1];    
  
  Body.set_arm_movement_velocity(
     vector.new({30,30,30,45,60,60})*math.pi/180);  
     
  t_last = unix.time();
end

turnAngleMag = 3*math.pi/180; --3 deg per sec


function update()
  t = unix.time();
  
  left_trigger = hcm:get_control_left_gripper()[1];
  right_trigger = hcm:get_control_right_gripper()[1];  
  turnAngleTarget = -10* (left_trigger-right_trigger)*math.pi/180;

  t_diff = t-t_last;
  angleDiff = turnAngleTarget - turnAngle;
  angleDiff = math.min(  turnAngleMag*t_diff, 
        math.max(-turnAngleMag*t_diff,  angleDiff));
  
  turnAngle = turnAngle + angleDiff;  
  turnAngle = math.min(10*math.pi/180,math.max(-10*math.pi/180,turnAngle));
  
  qLArm = Body.get_larm_command_position();
  qRArm = Body.get_rarm_command_position();

  calculate_arm_position();
  checkLeft = Body.check_larm_ik(trLArm);
  checkRight = Body.check_rarm_ik(trRArm);
  
  if checkLeft<0.01 and checkRight<0.01 then
    qLInv = Kinematics.inverse_l_arm(trLArm, qLArm);
    qRInv = Kinematics.inverse_r_arm(trRArm, qRArm);
    Body.set_larm_target_position(qLInv); 
    Body.set_rarm_target_position(qRInv);   
    hcm:set_wheel_turnangle(turnAngle); --Store current turnangle
  end
  t_last = t;
  
end

function exit()
  Body.set_arm_movement_velocity(
     vector.new({30,30,30,45,60,60})*math.pi/180);  

  print(_NAME..' Exit' ) 
end
