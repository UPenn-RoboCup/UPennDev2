module(..., package.seeall);
require('controller');

controller.wb_robot_init();
timeStep = controller.wb_robot_get_basic_time_step();
tDelta = .001*timeStep;
imuAngle = {0, 0, 0};
aImuFilter = 1 - math.exp(-tDelta/0.5);

-- Get webots tags:
tags = {};

-- THOR-OP joint names in webots
jointNames = {"neck_yaw","head_pitch",
              "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw","l_elbow_pitch","l_wrist_yaw","l_wrist_roll",
              "l_hip_yaw","l_hip_roll","l_hip_pitch", "l_knee_pitch","l_ankle_pitch","l_ankle_roll",
              "r_hip_yaw","r_hip_roll","r_hip_pitch", "r_knee_pitch","r_ankle_pitch","r_ankle_roll",
              "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw","r_elbow_pitch","r_wrist_yaw","r_wrist_roll",
	      "chest_yaw",
	      "l_wrist_grip1","l_wrist_grip2",
	      "r_wrist_grip1","r_wrist_grip2"
             };

nJoint = #jointNames;
indexHead = 1;			
nJointHead = 2;
indexLArm = 3;			--LArm: 3 4 5 6 7 8
nJointLArm = 6; 		
indexLLeg = 9;			--LLeg:9 10 11 12 13 14
nJointLLeg = 6;
indexRLeg = 15; 		--RLeg: 15 16 17 18 19 20
nJointRLeg = 6;
indexRArm = 21; 		--RArm: 21 22 23 24 25 26
nJointRArm = 6;
indexWaist = 27;
nJointWaist = 1;

indexLGrip = 28;
nJointLGrip = 2;
indexRGrip = 30;
nJointRGrip = 2;

jointReverse={
	5,7,  --LArm:  3 4 5 6 7 8
        --LLeg: 9 10 11 12 13 14
        --RLeg: 15 16 17 18 19 20
        23,25,--RArm: 21 22 23 24 25 26
	 --Waist: 27
	--Left gripper: 28 29
	--Right gripper: 30 31
}

jointBias={
        0,0,
	-90*math.pi/180,0,0,0,0,0,
	0,0,0,0,0,0,
	0,0,0,0,0,0,
	-90*math.pi/180,0,0,0,0,0,
	0,
	0,0,
	0,0,
}

moveDir={};
for i=1,nJoint do moveDir[i]=1; end
for i=1,#jointReverse do moveDir[jointReverse[i]]=-1; end

tags.joints = {};
for i,v in ipairs(jointNames) do
  tags.joints[i] = controller.wb_robot_get_device(v);
  controller.wb_servo_enable_position(tags.joints[i], timeStep);
end

tags.accelerometer = controller.wb_robot_get_device("Accelerometer");
controller.wb_accelerometer_enable(tags.accelerometer, timeStep);
tags.gyro = controller.wb_robot_get_device("Gyro");
controller.wb_gyro_enable(tags.gyro, timeStep);
tags.gps = controller.wb_robot_get_device("zero");
controller.wb_gps_enable(tags.gps, timeStep);
tags.compass = controller.wb_robot_get_device("compass");
controller.wb_compass_enable(tags.compass, timeStep);


controller.wb_robot_step(timeStep);
get_time = controller.wb_robot_get_time;

actuator = {};
actuator.command = {};
actuator.velocity = {};
actuator.position = {};
actuator.hardness = {};

for i = 1,nJoint do
  actuator.command[i] = 0;
  actuator.velocity[i] = 0;
  actuator.position[i] = 0;
  actuator.hardness[i] = 0;
end

function set_actuator_command(a, index)
  index = index or 1;
  if (type(a) == "number") then
    actuator.command[index] = moveDir[index]*(a+jointBias[index]);
  else
    for i = 1,#a do
      actuator.command[index+i-1] = moveDir[index+i-1]*(a[i]+jointBias[index+i-1]);
    end
  end
end

function set_actuator_velocity(a, index)
  index = index or 1;
  if (type(a) == "number") then
    actuator.velocity[index] = a;
  else
    for i = 1,#a do
      actuator.velocity[index+i-1] = a[i];
    end
  end
end

function set_actuator_hardness(a, index)
  index = index or 1;
  if (type(a) == "number") then
    actuator.hardness[index] = a;
  else
    for i = 1,#a do
      actuator.hardness[index+i-1] = a[i];
    end
  end
end

function get_sensor_position(index)
  if (index) then
    return moveDir[index]*controller.wb_servo_get_position(tags.joints[index])-jointBias[index];
  else
    local t = {};
    for i = 1,nJoint do
      t[i] = moveDir[i]*controller.wb_servo_get_position(tags.joints[i])-jointBias[i];
    end
    return t;
  end
end

function get_sensor_imuAngle(index)
  if (not index) then
    return imuAngle;
  else
    return imuAngle[index];
  end
end

-- Two buttons in the array
function get_sensor_button(index)
  local randThreshold = 0.001;
  if (math.random() < randThreshold) then
    return {1,0};
  else
    return {0,0};
  end
end

function get_head_position()
    local q = get_sensor_position();
    return {unpack(q, indexHead, indexHead+nJointHead-1)};
end
function get_larm_position()
  local q = get_sensor_position();
  return {unpack(q, indexLArm, indexLArm+nJointLArm-1)};
end
function get_rarm_position()
  local q = get_sensor_position();
  return {unpack(q, indexRArm, indexRArm+nJointRArm-1)};
end
function get_lleg_position()
  local q = get_sensor_position();
  return {unpack(q, indexLLeg, indexLLeg+nJointLLeg-1)};
end
function get_rleg_position()
  local q = get_sensor_position();
  return {unpack(q, indexRLeg, indexRLeg+nJointRLeg-1)};
end


function set_body_hardness(val)
  if (type(val) == "number") then
    val = val*vector.ones(nJoint);
  end
  set_actuator_hardness(val);
end
function set_head_hardness(val)
  if (type(val) == "number") then
    val = val*vector.ones(nJointHead);
  end
  set_actuator_hardness(val, indexHead);
end
function set_larm_hardness(val)
  if (type(val) == "number") then
    val = val*vector.ones(nJointLArm);
  end
  set_actuator_hardness(val, indexLArm);
end
function set_rarm_hardness(val)
  if (type(val) == "number") then
    val = val*vector.ones(nJointRArm);
  end
  set_actuator_hardness(val, indexRArm);
end
function set_lleg_hardness(val)
  if (type(val) == "number") then
    val = val*vector.ones(nJointLLeg);
  end
  set_actuator_hardness(val, indexLLeg);
end
function set_rleg_hardness(val)
  if (type(val) == "number") then
    val = val*vector.ones(nJointRLeg);
  end
  set_actuator_hardness(val, indexRLeg);
end
function set_waist_hardness( val )
  set_actuator_hardness(val, indexWaist);
end



function set_head_command(val)
  set_actuator_command(val, indexHead);
end
function set_lleg_command(val)
  set_actuator_command(val, indexLLeg);
end
function set_rleg_command(val)
  set_actuator_command(val, indexRLeg);
end
function set_larm_command(val)
  set_actuator_command(val, indexLArm);
end
function set_rarm_command(val)
  set_actuator_command(val, indexRArm);
end
function set_waist_command(val)
  set_actuator_command(val, indexWaist);
end

function update()
  -- Set actuators
  for i = 1,nJoint do
    if actuator.hardness[i] > 0 then
      if actuator.velocity[i] > 0 then
        local delta = actuator.command[i] - actuator.position[i];
        local deltaMax = tDelta*actuator.velocity[i];
        if (delta > deltaMax) then
          delta = deltaMax;
        elseif (delta < -deltaMax) then
          delta = -deltaMax;
        end
        actuator.position[i] = actuator.position[i]+delta;
      else
	    actuator.position[i] = actuator.command[i];
      end
      controller.wb_servo_set_position(tags.joints[i],
                                        actuator.position[i]);
    end
  end
  update_IMU();
  if (controller.wb_robot_step(timeStep) < 0) then
    --Shut down controller:
    os.exit();
  end
end

function update_IMU()
    
  acc=get_sensor_imuAcc();
  gyr=get_sensor_imuGyrRPY();

  local tTrans = Transform.rotZ(imuAngle[3]);
  tTrans= tTrans * Transform.rotY(imuAngle[2]);
  tTrans= tTrans * Transform.rotX(imuAngle[1]);

  gyrFactor = 0.85; --Empirical compensation value 
  gyrDelta = vector.new(gyr)*math.pi/180*tDelta*gyrFactor;

  local tTransDelta = Transform.rotZ(gyrDelta[3]);
  tTransDelta= tTransDelta * Transform.rotY(gyrDelta[2]);
  tTransDelta= tTransDelta * Transform.rotX(gyrDelta[1]);

  tTrans=tTrans*tTransDelta;
  imuAngle = Transform.getRPY(tTrans);

  local accMag = math.sqrt(acc[1]^2+acc[2]^2+acc[3]^2);
  if accMag>0.8 and accMag<1.1 then
--    print("Acc:",unpack(acc))
    --SJ: Corrected these 
    local angP=math.asin(-acc[1]/accMag);
    local angR=math.asin(acc[2]/accMag);
    imuAngle[1] = imuAngle[1] + aImuFilter*(angR - imuAngle[1]);
    imuAngle[2] = imuAngle[2] + aImuFilter*(angP - imuAngle[2]);
  end

  compass=get_sensor_compass();
  imuAngle[3] = math.atan2(compass[2],compass[1]);

--  print("Robot RPY:",imuAngle[1]*180/math.pi,imuAngle[2]*180/math.pi,imuAngle[3]*180/math.pi);

end


-- Extra for compatibility
function set_syncread_enable(val)
end

function set_actuator_eyeled( val )
end

function get_sensor_imuGyr0( )
  return vector.zeros(3)
end

function get_sensor_imuGyr( )
  return get_sensor_imuGyrRPY();
end

--Roll, Pitch, Yaw in degree per seconds unit
function get_sensor_imuGyrRPY( )
  gyro = controller.wb_gyro_get_values(tags.gyro);
--Roll, pitch corrected for THOROP model
  gyro_proc={(gyro[1]-512)/0.273, (gyro[2]-512)/0.273,(gyro[3]-512)/0.273};
  return gyro_proc;
end


function get_sensor_imuAcc( )
  accel = controller.wb_accelerometer_get_values(tags.accelerometer);
--Checked with THOROP model
  accel_proc = {(accel[1]-512)/128,(accel[2]-512)/128,-(accel[3]-512)/128};
  return accel_proc;
end

function get_sensor_gps()
--Checked with THOROP model and world
  gps = controller.wb_gps_get_values(tags.gps);
  return {gps[1],-gps[3],gps[2]};
end

function get_sensor_compass()
--Checked with THOROP model and the world
  compass = controller.wb_compass_get_values(tags.compass);
  return {compass[1],-compass[2],compass[3]};

end


function set_actuator_eyeled(color)
end

function set_actuator_headled(color)
end

function set_indicator_state(color)
end

function set_indicator_team(teamColor)
end

function set_indicator_kickoff(kickoff)
end

function set_indicator_batteryLevel(level)
end

function set_indicator_role(role)
end

function set_indicator_ball(color)
end

function set_indicator_goal(color)
end

function get_battery_level()
  return 10;
end

function get_change_state()
  return 0;
end

function get_change_enable()
  return 0;
end

function get_change_team()
  return 0;
end

function get_change_role()
  return 0;
end

function get_change_kickoff()
  return 0;
end

function set_actuator_us()
end

function get_sensor_usLeft()
  return vector.zeros(10);
end

function get_sensor_usRight()
  return vector.zeros(10);
end

function set_lleg_slope(val)
end

function set_rleg_slope(val)
end

function set_lleg_slope(val)
end
function set_rleg_slope(val)
end




-- Gripper method needed
function set_l_gripper_hardness(val)
  set_actuator_hardness(val, indexLGrip);
end

function set_r_gripper_hardness(val)
  set_actuator_hardness(val, indexRGrip);
end

function set_l_gripper_command(val)
  set_actuator_command(val, indexLGrip);
end

function set_r_gripper_command(val)
  set_actuator_command(val, indexRGrip);
end

function set_aux_hardness(val)
end

function set_aux_command(val)
end



