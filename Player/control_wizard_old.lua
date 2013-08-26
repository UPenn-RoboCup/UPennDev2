-----------------------------------------------
-- VERY simple control manager for tele-op
-- Receives UDP packet from controller 
--   and updates hcm for control
-- Also reads current joint angles 
-- And send them through udp 
-- SJ, 2013
--
-- This should run ~10hz 
----------------------------------------

dofile('include.lua')
local Config = require'ConfigPenn'
local Body = require(Config.Body);
local unix = require 'unix'
local jcm = require('jcm')
local hcm = require('hcm')
local scm = require('scm')
local mp = require'msgpack'
local udp = require 'udp'
local simple_ipc = require 'simple_ipc'

local hri_udp_recv = udp.new_receiver( Config.udp.PORT_CONTROL );
local feedback_channel_udp = udp.new_sender(Config.udp.UDP_IP, Config.udp.PORT_FEEDBACK);



local function process_message( cmd )
	-- If not ignited, then start the fire
	if hcm:get_control_ignited()[1]==0 and cmd.control_ignited==1 then
		-- Startup
		hcm:set_control_ignited(1)
	elseif hcm:get_control_ignited()[1]==1 and cmd.control_ignited==0 then
		-- Shutdown
		hcm:set_control_ignited(0)
	end

  hcm:set_control_wheel_velocity(cmd.control_wheel_velocity);
  hcm:set_control_head_movement(cmd.control_head_movement);
  hcm:set_control_lidar_active(cmd.control_lidar_active);

  hcm:set_control_left_arm_mode(cmd.control_left_arm_mode);
  hcm:set_control_left_arm_movement(cmd.control_left_arm_movement);
  hcm:set_control_left_gripper(cmd.control_left_gripper);

  hcm:set_control_right_arm_mode(cmd.control_right_arm_mode);
  hcm:set_control_right_arm_movement(cmd.control_right_arm_movement);
  hcm:set_control_right_gripper(cmd.control_right_gripper);

  hcm:set_control_emergency_stop(cmd.control_emergency_stop);
end

local function process_udp_hri()
  -- Loop through the UDP buffer of commands
  while hri_udp_recv:size()>0 do
    local hri_data = hri_udp_recv:receive()
    -- Unpack the data
    local command = mp.unpack( hri_data )
    local result = process_message( command )
    --print('\tControl input | Received', result )
  end
end

local function send_feedback()
  local jangles = jcm.get_position();
  local data={};
  data.larmangle = Body.get_larm_position();
  data.rarmangle = Body.get_rarm_position();
  data.waistangle = Body.get_waist_position();
  data.neckangle = Body.get_neck_position();
  data.grippers =  Body.get_gripper_position();

  pose_odom=scm:get_pose_odom();
  pose = scm:get_pose();
  pose_slam = scm:get_pose_slam();
  
  data.pose_odom = {pose_odom[1],pose_odom[2],pose_odom[3]};
  data.pose = {pose[1],pose[2],pose[3]};
  data.pose_slam = {pose_slam[1],pose_slam[2],pose_slam[3]};
  data.battery = jcm:get_battery();    
  local datapacked = msgpack.pack(data);
  local ret = feedback_channel_udp:send(datapacked);
--  print("Message sent:",ret)
end

local hri_udp_recv_poll = {}
hri_udp_recv_poll.socket_handle = hri_udp_recv:descriptor()
hri_udp_recv_poll.callback = process_udp_hri;

-- Poll multiple sockets
local wait_channels = { hri_udp_recv_poll }
local channel_poll = simple_ipc.wait_on_channels( wait_channels );

-- Update the wheels AT LEAST at 10Hz
local update_rate = 10 -- Hz
local channel_timeout = (1/update_rate)*1000 -- ms
local cnt = 0
t=unix.time();
tLast = t;
update_interval = 0.2; --5fps display refresh

while true do
local npoll = channel_poll:poll(channel_timeout)
--  unix.usleep(1E6*0.01);
  send_feedback();
  cnt = cnt+1;
  t=unix.time();
  if t-tLast > update_interval then
    os.execute('clear');
    print(string.format("Control manager | %d fps",cnt/(t-tLast) ));
    print(string.format("Wheel velocity: %d %d ",
      unpack(hcm:get_control_wheel_velocity())  ));

    print( string.format('\nArm:: LPos: %d %d %d',
       unpack(hcm:get_control_left_arm_movement()) ));
    print( string.format('Arm:: RPos: %d %d %d\n',
       unpack(hcm:get_control_right_arm_movement()) ));
		print("Ignited",hcm:get_control_ignited()[1])
    cnt = 0;
    tLast = t;
  end
end
