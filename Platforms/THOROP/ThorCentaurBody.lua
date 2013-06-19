module(..., package.seeall);
local jcm = require('jcm');

require('math')
require('Kinematics')
require('vector')
require('util')

------------------------------------------------
-- Temporary body function for THOR-OP platform
-- Mainly does the joint angle transform
------------------------------------------------

local RAD = math.pi/180;

--Servo IDs for each limb
LArmServoID = {1,3,5,7,9,11};
RArmServoID = {2,4,6,8,10,12};
LFingerServoID = {14,16,18};
RFingerServoID = {13,15,17};
WheelServoID = {20,19};
NeckServoID = {27,28};
WaistServoID = {25,26};
LidarServoID = {36};

LArmJointDir={1,1,1,-1,1,-1,};
RArmJointDir={-1,1,1,1, 1,-1,};
WheelJointDir = {1,-1};
NeckJointDir = {1,1};
WaistJointDir = {1,1};
LidarJointDir = {-1};

--[[
LArmJointBias = {-math.pi/2,-math.pi/2,-math.pi/2,0,math.pi/2,0,};
RArmJointBias = {-math.pi/2,math.pi/2,math.pi/2,0,-math.pi/2,0};
--]]

--After reflashing the servo
LArmJointBias = {-math.pi/2,-math.pi/2,-math.pi/2,math.pi/4,math.pi/2,0,};
RArmJointBias = {-math.pi/2,math.pi/2,math.pi/2,math.pi/4,-math.pi/2,0};


NeckJointBias = {0,-1.8*RAD};




LidarJointBias = {-math.pi}

LFingerOpen = {147,218,143};
LFingerClose = {186,177,186};

RFingerOpen = {208,63,215};
RFingerClose = {174,106,172};


dArmVelAngle = vector.new({20,20,20,20,60,60})*math.pi/180; --30 degree per second
dArmVelTransform = {0.05,0.05,0.05, --5 cm per second
		60*math.pi/180,60*math.pi/180,60*math.pi/180};
--Faster
dArmVelAngle = vector.new({20,20,20,30,90,90})*math.pi/180; --30 degree per second


--super slow
dArmVelAngle = vector.new({10,10,10,15,45,45})*math.pi/180; --30 degree per second







dWaistVelAngle = vector.new({10,10})*RAD; --super slow
dNeckVelAngle = vector.new({90,90})*RAD;
dWheelVel = 6000; --2000 unit per sec

local tLastUpdate = 0;

larm_moving_linear = false;
rarm_moving_linear = false;
larm_moving = false;
rarm_moving = false;
emergency_stop = false;


function larm_linear_movement_done() return not larm_moving_linear; end
function rarm_linear_movement_done() return not rarm_moving_linear; end
function larm_joint_movement_done() 
  local qLArm = get_larm_command_position();
  local errSum = 0;
  for i=1,#qLArm do
    dError = (qLArmTarget[i]-qLArm[i]) % (2*math.pi);
    if (dError > math.pi) then dError = dError - 2*math.pi;end
    errSum = errSum + math.abs(dError);
  end
  if errSum<0.5*RAD then return true;
  else return false; end
end
function rarm_joint_movement_done() 
  local qRArm = get_rarm_command_position();
  local errSum = 0;
  for i=1,#qRArm do
    dError = (qRArmTarget[i]-qRArm[i]) % (2*math.pi);
    if (dError > math.pi) then dError = dError - 2*math.pi;end
    errSum = errSum + math.abs(dError);
  end
  if errSum<0.5*RAD then return true;
  else return false; end
end

function stop_movement() emergency_stop = true; end
function enable_larm_linear_movement(val) larm_moving_linear = val; end
function enable_rarm_linear_movement(val) rarm_moving_linear = val; end
function set_arm_movement_velocity(v)   dArmVelAngle = v; end

function check_larm_ik(tr)
  qLArm = get_larm_position();
  qInv = Kinematics.inverse_l_arm(tr, qLArm);
  torso_arm_ik = Kinematics.l_arm_torso(qInv);
  dist = math.sqrt(
    (torso_arm_ik[1]-tr[1])^2+
    (torso_arm_ik[2]-tr[2])^2+
    (torso_arm_ik[3]-tr[3])^2);
    
  dist_angle = math.sqrt(
    util.mod_angle( torso_arm_ik[4]-tr[4] )^2+
    util.mod_angle( torso_arm_ik[5]-tr[5])^2+
    util.mod_angle( torso_arm_ik[6]-tr[6])^2);

  return dist,dist_angle;
end

function check_rarm_ik(tr)
  qRArm = get_rarm_position();
  qInv = Kinematics.inverse_r_arm(tr, qRArm);
  torso_arm_ik = Kinematics.r_arm_torso(qInv);
  dist = math.sqrt(
    (torso_arm_ik[1]-tr[1])^2+
    (torso_arm_ik[2]-tr[2])^2+
    (torso_arm_ik[3]-tr[3])^2);

  dist_angle = math.sqrt(
    util.mod_angle( torso_arm_ik[4]-tr[4] )^2+
    util.mod_angle( torso_arm_ik[5]-tr[5])^2+
    util.mod_angle( torso_arm_ik[6]-tr[6])^2);

  return dist,dist_angle;
end

function set_larm_target_transform(tr)
  if check_larm_ik(tr)<0.001 then
    trLArmTarget = tr;
    return true;
  else
    return false;
  end
end

function set_rarm_target_transform(tr)
  if check_rarm_ik(tr)<0.001 then
    trRArmTarget = tr;
    return true;
  else
    return false;
  end
end


function update_larm_linear_movement(tdiff)
  if larm_moving_linear then
    --Left arm linear movement
    local qLArm = get_larm_command_position();
    local trLArm = Kinematics.l_arm_torso(qLArm);
    local errSum = 0;
    for i=1,6 do
      dError = trLArmTarget[i]-trLArm[i];
      errSum = errSum + math.abs(dError);
      dError = math.max(-dArmVelTransform[i]*tdiff,
		 math.min(dArmVelTransform[i]*tdiff,dError));
      trLArm[i] = trLArm[i] + dError;
    end
    local qInv = Kinematics.inverse_l_arm(trLArm, qLArm);
    if check_larm_ik(trLArm)<0.001 then
      qLArmTarget = qInv;
    else --Stuck somewhere... stop movement
  --    print("STUCK, MOVEMENT STOP")
      larm_moving_linear = false; 
    end
  else --Update target transform according to current command angle
    local qLArm = get_larm_command_position();
    trLArmTarget = Kinematics.l_arm_torso(qLArm);
  end
end

function update_rarm_linear_movement(tdiff)
  --Right arm linear movement
  if rarm_moving_linear then
    local errSum = 0;
    local qRArm = get_rarm_command_position();
    local trRArm = Kinematics.r_arm_torso(qRArm);
    for i=1,6 do
      dError = trRArmTarget[i]-trRArm[i];
      errSum = errSum + math.abs(dError);
      dError = math.max(-dArmVelTransform[i]*tdiff,
		 math.min(dArmVelTransform[i]*tdiff,dError));
      trRArm[i] = trRArm[i] + dError;
    end
    local qInv = Kinematics.inverse_r_arm(trRArm, qRArm);
    if check_rarm_ik(trRArm)<0.001 then
      qRArmTarget = qInv;
    else --Stuck somewhere... stop movement
      rarm_moving_linear = false; 
    end
--    if errSum < 0.001 then 
--      rarm_moving_linear = false; --Reached target position
--    end 
  else --Update target transform according to current command angle
    local qRArm = get_rarm_command_position();
    trRArmTarget = Kinematics.r_arm_torso(qRArm);
  end
end

function update_joint_movement(tdiff)

  if emergency_stop then
    qLArm = get_larm_command_position();
    qRArm = get_rarm_command_position();
    qWaist = get_waist_command_position();
    qNeck = get_neck_command_position();

    set_larm_position(qLArm);
    set_rarm_position(qRArm);
    set_waist_position(qWaist);
    set_neck_position(qNeck);
    return;
  end

  --Left arm
  local qLArm = get_larm_command_position();
  for i=1,#qLArm do
    dError = (qLArmTarget[i]-qLArm[i]) % (2*math.pi);
    if (dError > math.pi) then dError = dError - 2*math.pi;end
    dError = math.max(-dArmVelAngle[i]*tdiff, math.min(dArmVelAngle[i]*tdiff,dError));
    qLArm[i] = qLArm[i] + dError;
  end
  set_larm_position(qLArm);
 

  --Right arm
  local qRArm = get_rarm_command_position();
  for i=1,#qRArm do
    dError = (qRArmTarget[i]-qRArm[i]) % (2*math.pi);
    if (dError > math.pi) then dError = dError - 2*math.pi;end
    dError = math.max(-dArmVelAngle[i]*tdiff,math.min(dArmVelAngle[i]*tdiff,dError));
    qRArm[i] = qRArm[i] + dError;
  end
  set_rarm_position(qRArm);

  --Waist
  local qWaist = get_waist_command_position();
  for i=1, #qWaist do
    dError = (qWaistTarget[i]-qWaist[i]) % (2*math.pi);
    if (dError > math.pi) then dError = dError - 2*math.pi;end
    dError = math.max(-dWaistVelAngle[i]*tdiff, math.min(dWaistVelAngle[i]*tdiff,dError));
    qWaist[i] = qWaist[i] + dError;
  end
  set_waist_position(qWaist);

  --Neck
  local qNeck = get_neck_command_position();
  for i=1, #qNeck do
    dError = (qNeckTarget[i]-qNeck[i]) % (2*math.pi);
    if (dError > math.pi) then dError = dError - 2*math.pi;end
    dError = math.max(-dNeckVelAngle[i]*tdiff,math.min(dNeckVelAngle[i]*tdiff,dError));
    qNeck[i] = qNeck[i] + dError;
  end
  set_neck_position(qNeck);

  --Wheels
  local vWheel = get_wheel_command_velocity();
  for i=1, #vWheel do
    vError = vWheelTarget[i]-vWheel[i];
    vError = math.max(-dWheelVel*tdiff,
		math.min(dWheelVel*tdiff,vError));
    vWheel[i] = vWheel[i] + vError;
  end
  set_command_velocity_id(WheelServoID,vWheel,WheelJointDir);

end


------------------
-- Set functions
------------------

function set_larm_target_position(q) qLArmTarget = q; end
function set_rarm_target_position(q) qRArmTarget = q; end
function set_neck_target_position(q) qNeckTarget = q; end
function set_waist_target_position(q)   qWaistTarget = q;end

function set_larm_position(q)  --q in radian
  set_command_position_id_converted(LArmServoID,q,LArmJointDir, LArmJointBias)
end

function set_rarm_position(q)  --q in radian
  set_command_position_id_converted(RArmServoID,q,RArmJointDir, RArmJointBias)
end

function set_neck_position(q)  --q in radian
  set_command_position_id_converted(NeckServoID,q,NeckJointDir, NeckJointBias)
end

function set_waist_position(q)  --q in radian
  set_command_position_id_converted(WaistServoID,q,WaistJointDir, {0,0})
end

function set_lidar_position(q)
  set_command_position_id_converted(LidarServoID,q,LidarJointDir,LidarJointBias)
end

function set_lwheel_velocity(v)  
  vWheelTarget[1] = v;
end
function set_rwheel_velocity(v)  
  vWheelTarget[2] = v;
end

function set_wheel_velocity_direct(v)  
  vWheelTarget=v;
  set_command_velocity_id(WheelServoID,vWheelTarget,WheelJointDir);
end




function set_lhand_position(v) --0 fully open, 1 fully close
  qFinger = {};
  qFinger[1] = LFingerOpen[1] * (1-v) + LFingerClose[1] * v;  
  qFinger[2] = LFingerOpen[2] * (1-v) + LFingerClose[2] * v;  
  qFinger[3] = LFingerOpen[3] * (1-v) + LFingerClose[3] * v;  
  set_command_position_id(LFingerServoID,qFinger)
  
  grip=jcm:get_gripper();
  jcm:set_gripper({v,grip[2]});    
end

function set_rhand_position(v)
  qFinger = {};
  qFinger[1] = RFingerOpen[1] * (1-v) + RFingerClose[1] * v;  
  qFinger[2] = RFingerOpen[2] * (1-v) + RFingerClose[2] * v;  
  qFinger[3] = RFingerOpen[3] * (1-v) + RFingerClose[3] * v;  
  set_command_position_id(RFingerServoID,qFinger)
  
  grip=jcm:get_gripper();
  jcm:set_gripper({grip[1], v});    
end

function get_gripper_position()
  return jcm:get_gripper();
end

------------------
-- Get functions
------------------

function get_larm_position()
  return get_position_id_converted(LArmServoID, LArmJointDir, LArmJointBias);
end

function get_rarm_position()
  return get_position_id_converted(RArmServoID, RArmJointDir, RArmJointBias);
end

function get_larm_command_position()
  return get_command_position_id_converted(LArmServoID, LArmJointDir, LArmJointBias);
end

function get_rarm_command_position()
  return get_command_position_id_converted(RArmServoID, RArmJointDir, RArmJointBias);
end

function get_neck_position() 
  return get_position_id_converted(NeckServoID,NeckJointDir,NeckJointBias)
end

function get_waist_position()  
  return get_position_id_converted(WaistServoID,WaistJointDir, {0,0})
end

function get_neck_command_position() 
  return get_command_position_id_converted(NeckServoID,NeckJointDir,NeckJointBias)
end

function get_waist_command_position() 
  return get_command_position_id_converted(WaistServoID,WaistJointDir, {0,0})
end

function get_wheel_command_velocity() 
  return get_command_velocity_id(WheelServoID,WheelJointDir)
end

function get_lidar_position()
  return get_position_id_converted(LidarServoID,LidarJointDir,LidarJointBias)
end

function get_wheel_position()
  return get_position_id_converted(WheelServoID,WheelJointDir,{0,0})
end


function set_command_velocity_id(ids,val,dir)
  local vel = jcm:get_command_velocity();
  for j,v in ipairs(ids) do vel[v] = val[j]*dir[j];  end
  jcm:set_command_velocity(vel);
end

function set_command_position_id(ids,val)
  local pos = jcm:get_command_position();
  for j,v in ipairs(ids) do 
     pos[v] = val[j];
  end
  jcm:set_command_position(pos);
end

function set_command_position_id_converted(ids,val,dir,bias)
  local pos = jcm:get_command_position();
  for j,v in ipairs(ids) do 
     pos[v] = dir[j] * (val[j]+bias[j]) / RAD;  
  end
  jcm:set_command_position(pos);
end

function get_position_id_converted(ids, dir, bias)
  local pos = jcm:get_position();
  ret={};
  for j,val in ipairs(ids) do 
    ret[j] = pos[val]*RAD * dir[j] - bias[j]; 
  end
  return ret;
end

function get_command_position_id_converted(ids, dir, bias)
  local pos = jcm:get_command_position();
  ret={};
  for j,val in ipairs(ids) do 
    ret[j] = pos[val]*RAD * dir[j] - bias[j]; 
  end
  return ret;
end

function get_command_velocity_id(ids,dir)
  local vel = jcm:get_command_velocity();
  ret={};
  for j,val in ipairs(ids) do ret[j] = vel[val]*dir[j]; end
  return ret;
end

function calculate_odometry()
  qWheel = get_wheel_position();
  qWheelDiff = {
    util.mod_angle(qWheel[1]-qWheelOld[1]),
    util.mod_angle( qWheel[2]-qWheelOld[2])};
  qWheelOld[1],qWheelOld[2] = qWheel[1],qWheel[2];      
  local wheel_radius = 0.105;  
  local wheel_width = 0.34;

  --Simple odometry for now
  local dx = (qWheelDiff[1]+qWheelDiff[2])/2 * wheel_radius;
  local da = (qWheelDiff[2]-qWheelDiff[1])/2 *wheel_radius / (wheel_width/2);
      
  pose = util.pose_global({dx,0,da} , pose);     
end

function get_odometry()
  return pose;
end

function reset_odometry(pose0)
  pose = {pose0[1],pose0[2],pose0[3]};
end

function update_movement()
  local t = unix.time();
  if tLastUpdate==0 then tLastUpdate = t; end
  local tdiff = t-tLastUpdate;  
  update_larm_linear_movement(tdiff);
  update_rarm_linear_movement(tdiff);
  update_joint_movement(tdiff);
  calculate_odometry(tdiff);    
  tLastUpdate = t;
end

qLArm = get_larm_position();
qRArm = get_rarm_position();
qLArmTarget = get_larm_position();
qRArmTarget = get_rarm_position();
qNeckTarget = get_neck_position();
qWaistTarget = get_waist_position();
vWheelTarget = {0,0};
vWheel = {0,0};
qWheel = get_wheel_position();
qWheelOld = get_wheel_position();

pose = {0,0,0};

--trLArmTarget = Kinematics.l_arm_torso(qLArmTarget);
--trRArmTarget = Kinematics.r_arm_torso(qRArmTarget);
