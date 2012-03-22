module(..., package.seeall);
require('controller');
require('Kinematics');

controller.wb_robot_init();
timeStep = controller.wb_robot_get_basic_time_step();
tDelta = .001*timeStep;

-- Get webots tags:
tags = {};

--joint names:
jointNames = {"Head","Neck",
              "L_Shoulder_Pitch", "L_Shoulder_Roll", "L_Shoulder_Yaw","L_Elbow",
              "L_Hip_Yaw", "L_Hip_Roll", "L_Hip_Pitch", "L_Knee_Pitch", "L_Ankle_Pitch", "L_Ankle_Roll",
              "R_Hip_Yaw", "R_Hip_Roll", "R_Hip_Pitch", "R_Knee_Pitch", "R_Ankle_Pitch", "R_Ankle_Roll",
              "R_Shoulder_Pitch", "R_Shoulder_Roll", "R_Shoulder_Yaw","R_Elbow",
             };

nJoint = #jointNames;

indexHead = 1;			
nJointHead = 2;
indexLArm = 3;			--LArm: 3 4 5 6
nJointLArm = 4; 		
indexLLeg = 7;			--LLeg:7 8 9 10 11 12
nJointLLeg = 6;
indexRLeg = 13; 		--RLeg: 13 14 15 16 17 18
nJointRLeg = 6;
indexRArm = 19; 		--RArm: 19 20 21 22
nJointRArm = 4;
indexWaist = 23;
nJointWaist = 0;

jointReverse={
	 3,4,5,6,--LArm:  3 4 5 6
         --LLeg: 7 8 9 10 11 12
	 --RLeg: 13 14 15 16 17 18
	 22,--RArm: 19 20 21 22
}

jointBias={
        0,0,
	-90*math.pi/180,0,0,0,
	0,0,0,0,0,0,
	0,0,0,0,0,0,
	-90*math.pi/180,0,0,0,
}

moveDir={};
for i=1,nJoint do moveDir[i]=1; end
for i=1,#jointReverse do moveDir[jointReverse[i]]=-1; end

--servo names

servoNames = {"Head","Neck",
              "L_Shoulder_Pitch", "L_Shoulder_Roll", "L_Shoulder_Yaw","L_Elbow",
	     "l_hipyaw", "li_hip_servo", "lo_hip_servo","l_knee_servo", "li_ankle_servo", "lo_ankle_servo",
   	     "r_hipyaw", "ri_hip_servo", "ro_hip_servo", "r_knee_servo", "ri_ankle_servo", "ro_ankle_servo",              
	     "R_Shoulder_Pitch", "R_Shoulder_Roll", "R_Shoulder_Yaw","R_Elbow",
             };

nServo = #servoNames;

servoReverse={
  8,9,10,11,12,  --LLeg: 7 8 9 10 11 12
  14,15,16,17,18, --RLeg: 13 14 15 16 17 18
}

servoBias={
  0,0, 
  0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,
}

servoDir={};

servoType={ --0 for rot, 1 for linear
  0,0,
  0,0,0,0,
  0,1,1,1,1,1,
  0,1,1,1,1,1,
  0,0,0,0,
}

for i=1,nServo do servoDir[i]=1; end
for i=1,#servoReverse do servoDir[servoReverse[i]]=-1; end

controlP = 50;
maxForce = 1000;
maxVelocity = 10;
maxAcceleration = -1;
imuAngle = {0, 0, 0};
aImuFilter = 1 - math.exp(-tDelta/0.5);

tags.servo = {};
for i=1,#servoNames do
  tags.servo[i] = controller.wb_robot_get_device(servoNames[i]);
  controller.wb_servo_enable_position(tags.servo[i], timeStep);
  if servoType[i]==1 then
    controller.wb_servo_enable_motor_force_feedback(tags.servo[i], timeStep)
    controller.wb_servo_set_control_p(tags.servo[i], controlP)
    controller.wb_servo_set_motor_force(tags.servo[i], maxForce)
    controller.wb_servo_set_velocity(tags.servo[i], maxVelocity)
    controller.wb_servo_set_acceleration(tags.servo[i], maxAcceleration)
    controller.wb_servo_set_force(tags.servo[i],0);
  end
end

tags.accelerometer = controller.wb_robot_get_device("accelerometer"); 
controller.wb_accelerometer_enable(tags.accelerometer, timeStep); 
tags.gyro = controller.wb_robot_get_device("gyro"); 
controller.wb_gyro_enable(tags.gyro, timeStep); 
tags.l_fts = controller.wb_robot_get_device('left_fts_receiver')
controller.wb_receiver_enable(tags.l_fts, timeStep)
tags.r_fts = controller.wb_robot_get_device('right_fts_receiver')
controller.wb_receiver_enable(tags.r_fts, timeStep)
tags.gps=controller.wb_robot_get_device("zero"); 
controller.wb_gps_enable(tags.gps, timeStep); 
tags.eyeled = controller.wb_robot_get_device("EyeLed"); 
controller.wb_led_set(tags.eyeled,0xffffff) tags.headled = 
controller.wb_robot_get_device("HeadLed"); 
controller.wb_led_set(tags.headled,0x00ff00);

controller.wb_robot_step(timeStep);
get_time = controller.wb_robot_get_time;

t0=get_time();

------------------------------- 
--TODO: Force control/feedback 
------------------------------

actuator = {};
actuator.command = {};
actuator.velocity = {};
actuator.position = {};
actuator.hardness = {};

sensor = {};
sensor.position = {};

for i = 1,nJoint do
  actuator.command[i] = 0;
  actuator.velocity[i] = 0;
  actuator.position[i] = 0;
  actuator.hardness[i] = 0;
  sensor.position[i] = 0;
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
    return sensor.position[index];
  else
    local t = {};
    for i = 1,nJoint do
      t[i] = sensor.position[i];
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
--    local q = get_sensor_position();
--    return {unpack(q, indexHead, indexHead+nJointHead-1)};
  return 0,0;
end
function get_larm_position()
--  local q = get_sensor_position();
--  return {unpack(q, indexLArm, indexLArm+nJointLArm-1)};
  return 0,0,0,0;
end
function get_rarm_position()
--  local q = get_sensor_position();
--  return {unpack(q, indexRArm, indexRArm+nJointRArm-1)};
  return 0,0,0,0;
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
--  set_actuator_hardness(val, indexWaist);
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
--  set_actuator_command(val, indexWaist);
end

function update_servo()
  --Servo IK handling for servo # 7-18
  local legCommand={}
  for i=1,nJointLLeg+nJointRLeg do
    legCommand[i]=actuator.position[indexLLeg+i-1];
  end
  local legServoCommand = Kinematics.inverse_joints(legCommand);

  local servoCommand={}
  for i=1,nServo do
    servoCommand[i]=actuator.position[i];
  end
  for i=1,nJointLLeg+nJointRLeg do
    servoCommand[indexLLeg+i-1]=legServoCommand[i];
  end
  for i=1,nServo do
      controller.wb_servo_set_position(tags.servo[i],
	servoCommand[i]*servoDir[i]+servoBias[i]);
  end
end

function update_sensor()
-- SJ: Instead of reading from servos and run FK to get joint angles,
-- Just use commanded joint angle for current sensor angles for now
-- TODO: FK based joint angle calculation
  for i=1,nJoint do
      sensor.position[i] = actuator.command[i];
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

  local accMag = acc[1]^2+acc[2]^2+acc[3]^2;
  if accMag>0.8 and accMag<1 then
    local angR=math.asin(-acc[2]);
    local angP=math.asin(acc[1]);
    imuAngle[1] = imuAngle[1] + aImuFilter*(angR - imuAngle[1]);
    imuAngle[2] = imuAngle[2] + aImuFilter*(angP - imuAngle[2]);
  end

--  print("RPY:",unpack(imuAngle*180/math.pi))
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
    end
  end
  --Update servo based on commanded joint angle
  update_servo();
  update_sensor();
  update_IMU();

  if (controller.wb_robot_step(timeStep) < 0) then
    os.exit()
  end
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
  --tested correct with saffir model
  gyro_proc=vector.new({(gyro[1]-512), (gyro[2]-512),(gyro[3]-512)})/0.273;
  return gyro_proc;
end

--X,Y,Z in g(9.8m/s^2)  unit
function get_sensor_imuAcc( ) 
  accel = controller.wb_accelerometer_get_values(tags.accelerometer);
  --tested with saffir model
  return (vector.new({-(accel[1]-512),-(accel[2]-512),-(accel[3]-512)})/128);
end

function get_sensor_gps()
  gps = controller.wb_gps_get_values(tags.gps);
  return gps;
end

function set_actuator_eyeled(color)
--input color is 0 to 31, so multiply by 8 to make 0-255
  code= color[1] * 0x80000 + color[2] * 0x800 + color[3]*8;
--  controller.wb_led_set(tags.eyeled,code)
end

function set_actuator_headled(color)
  --input color is 0 to 31, so multiply by 8 to make 0-255
  code= color[1] * 0x80000 + color[2] * 0x800 + color[3]*8;
  --  controller.wb_led_set(tags.headled,code)
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
  -- color is a 3 element vector
  -- convention is all zero indicates no detection
  if( color[1]==0 and color[2]==0 and color[3]==0 ) then
    set_actuator_eyeled({15,15,15});
  else
    set_actuator_eyeled({31*color[1],31*color[2],31*color[3]});
  end

end

function set_indicator_goal(color)
-- color is a 3 element vector
  -- convention is all zero indicates no detection
  if( color[1]==0 and color[2]==0 and color[3]==0 ) then
    set_actuator_headled({15,15,15});
  else
    set_actuator_headled({31*color[1],31*color[2],31*color[3]});
  end

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
function set_gripper_hardness(val)
end

function set_gripper_command(val)
end




