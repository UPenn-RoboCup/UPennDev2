dofile('./include.lua')

Config = require('ConfigPenn')
local unix = require'unix'
local getch = require'getch'
local mp = require('msgpack');
local udp = require('udp');
local hcm = require'hcm' --HID shm
local scm = require'scm' --SLAM shm
local simple_ipc = require 'simple_ipc'

local command_udp_recv = udp.new_receiver( Config.udp.PORT_COMMAND );

require('vector')
require('ArmFSM')
require('BodyFSM')
require('HeadFSM')
require('LidarFSM')


require('bodyNavigate')
require('armGrabLeft')

RAD = math.pi/180;
local Body = require(Config.Body)
local maxFPS = Config.movement.maxFPS;

estop = {0};

local function entry()
  Body.set_lwheel_velocity(0);
  Body.set_rwheel_velocity(0);
  Body.set_neck_target_position({0,0})
  Body.set_waist_target_position({0,1.3*RAD})

  LidarFSM.entry();
  HeadFSM.entry();
  BodyFSM.entry();
  ArmFSM.entry();
  
  --Let's store wheel data here
  hcm:set_wheel_pos({0.30,0,0.01});
  hcm:set_wheel_radius(0.16);
  hcm:set_wheel_pitchangle(2*math.pi/180);
  hcm:set_wheel_turnangle(0);

end

local function update(tdiff)
  LidarFSM.update();
  HeadFSM.update();
  BodyFSM.update();
  ArmFSM.update();

  Body.update_movement();
  estop = hcm:get_control_emergency_stop();
  return;
end


local function process_udp_command()
  while command_udp_recv:size()>0 do
    t=unix.time();
    local command_data = command_udp_recv:receive()
    cmd = mp.unpack(command_data);
    
--SJ: the returned string has one more characters than original one
	  fsmname = string.sub(cmd.fsm,1,#cmd.fsm-1);
	  eventname = string.sub(cmd.event,1,#cmd.event-1);

    print(fsmname,eventname)

    if fsmname =="body" then
      BodyFSM.sm:add_event(eventname)
		  if eventname=="navigate" then		    
		    print("setting waypoint...")
		    slam_pose = scm:get_pose_slam();
		    Body.reset_odometry(slam_pose);
				bodyNavigate.set_waypoint(cmd.data);
			end
	  elseif fsmname=="head" then
      HeadFSM.sm:add_event(eventname)
	  elseif fsmname=="arm" then
      ArmFSM.sm:add_event(eventname)
		  if eventname=="init" then
		    BodyFSM.sm:add_event('stop')
		  elseif eventname=="reset" then
 		    BodyFSM.sm:add_event('teleop')
			  --cmd.data --0 for left, 1 for right
		  end
		  if eventname=="wheelgrab" then
 		    BodyFSM.sm:add_event('stop')

		    wheeldata = cmd.data;
				if #cmd.data>3 then
  		    --hack
			    wheeldata[1] = wheeldata[1]-0.01;
			    wheeldata[3] = wheeldata[3]-0.04;
	        hcm:set_wheel_pos({wheeldata[1],wheeldata[2],wheeldata[3]});
	        hcm:set_wheel_yawangle(wheeldata[4]);
	        hcm:set_wheel_pitchangle(wheeldata[5]);        
	        hcm:set_wheel_radius(wheeldata[6]);
	 		    print("Wheel data read")
	 		    print(string.format("pos:%.2f %.2f %.2f\nYaw:%.1f Pitch:%.1f Radius:%.1f",
	 		       wheeldata[1],wheeldata[2],wheeldata[3],
	 		       wheeldata[4]*180/math.pi,
	 		       wheeldata[5]*180/math.pi,
	 		       wheeldata[6] ));
				end
		  end
    end
  end
end

local command_udp_recv_poll = {}
command_udp_recv_poll.socket_handle = command_udp_recv:descriptor()
command_udp_recv_poll.callback = process_udp_command;
local wait_channels = {command_udp_recv_poll}
local channel_poll = simple_ipc.wait_on_channels( wait_channels );


local inv_fps = 1/maxFPS
local cnt = 0
local t_start = unix.time()
local t_last = t_start
local t_debug = t_start
local t = t_last

update_time = 0;
awake_ratio = 0;
debug_interval = 0.5;
debug_interval = 2;

debug_interval = 10;



entry()
while estop[1]==0 do
  -- Timing
  t_last = t;
  t = unix.time()
  t_diff = t - t_last;
  if t_diff < inv_fps then
    sleeptime = (inv_fps - t_diff);
    unix.usleep(1E6* sleeptime  )
    t = unix.time()
    t_diff = t - t_last;
  else
    sleeptime = 0;
  end
  awake_ratio = awake_ratio + (1-sleeptime/t_diff);	
  update(t_diff);

--sj: 10ms poll
  npoll = channel_poll:poll(10)

  cnt = cnt+1;
  pose=Body.get_odometry();
  scm:set_pose_odom(pose);    
  if Config.use_odometry_only then
    scm:set_pose(pose);
  end
  -- Debugging: Once a second
  local t_diff_debug = t - t_debug
  if t_diff_debug > debug_interval then
--    os.execute('clear');
    print( string.format('\nMovement | Updating at %.2f FPS awake %.1f %%',
	    cnt/t_diff_debug, awake_ratio/cnt*100) )
    print( string.format('Current odometry: %.3f %.3f %.1f', 
      pose[1],pose[2],pose[3]/RAD ));
    print( string.format('Current state: %s  %s  %s  %s\n', 
			HeadFSM.sm:get_state(),LidarFSM.sm:get_state(),
			BodyFSM.sm:get_state(),ArmFSM.sm:get_state() ));
    t_debug = t;
    cnt = 0
    awake_ratio = 0;
  end
end

