dofile('./include.lua')
Config = require('ConfigPenn')

require('unix')

local hcm = require'hcm'
local xbox360 = require 'xbox360'
local simple_ipc = require'simple_ipc'
local msgpack = require 'msgpack'


--[[
Buttons
1: D-Pad up  D-Pad up
2: D-Pad down  D-Pad down
3: D-Pad left  D-Pad left
4: D-pad right   D-Pad right
5: Start button  Button 8
6: Back button   Button 7
7: Left stick press  Button 9
8: Right stick press   Button 10
9: Button LB   Button 5
10: Button RB  Button 6
11: Xbox logo button
12: Unused
13: Button A   Button 1
14: Button B   Button 2
15: Button X   Button 3
16: Button Y   Button 4

Axes
1  Left trigger  Z-axis down
2  Right trigger   Z-axis up
3  Left stick X-axis   X-axis
4  Left stick Y-axis   Y-axis
5  Right stick X-axis  X-turn
6  Right stick Y-axis  Y-turn
7  Unused
--]]

xbox360.open()
udp_port = Config.udp.PORT_CONTROL;


LVel = 0;
RVel = 0;
lidar_active = 0;
old_button = xbox360.button();
controller_channel = simple_ipc.new_publisher('controller') --controller
local udp = require 'udp'
controller_channel_udp = udp.new_sender(Config.udp.UDP_IP, udp_port);
local t_last_udp = unix.time()

-- NEVER START IGNITED
hcm:set_control_ignited(0);
--Control struct
control_data={};
control_data.control_ignited = vector.zeros(1);
control_data.control_wheel_velocity = vector.zeros(2);
control_data.control_head_movement = vector.zeros(2);
control_data.control_lidar_active = vector.zeros(1);
control_data.control_left_arm_movement = vector.zeros(3);
control_data.control_right_arm_movement = vector.zeros(3);
control_data.control_left_arm_mode = vector.zeros(1);
control_data.control_right_arm_mode = vector.zeros(1);
control_data.control_left_gripper = vector.zeros(1);
control_data.control_right_gripper = vector.zeros(1);
control_data.control_emergency_stop = vector.zeros(1);

function sign(x)
 if x>0 then return 1;
 else return -1;
 end
end


function update()
  local axes = xbox360.axis() --LT RT LX LY RX RY, 128 MAX
  local button = xbox360.button()

  left={};
  right={};
  left.analog = {
	((axes[3]+128)%256-128) / 128,
	((axes[4]+128)%256-128) / 128,
	 axes[1]/256
	};
  right.analog = {
	((axes[5]+128)%256-128) / 128,
	((axes[6]+128)%256-128) / 128,
	 axes[2]/256
	};
  --deadband handling 
  deadband = 0.4;


  left.analog[1] = math.max(0,math.abs(left.analog[1])-deadband)
	*sign(left.analog[1]) / (1-deadband);
  left.analog[2] = math.max(0,math.abs(left.analog[2])-deadband)
	*sign(left.analog[2]) / (1-deadband);
  right.analog[1] = math.max(0,math.abs(right.analog[1])-deadband)
	*sign(right.analog[1]) / (1-deadband);
  right.analog[2] = math.max(0,math.abs(right.analog[2])-deadband)
	*sign(right.analog[2]) / (1-deadband);


  max_speed =9000;

  --Left analog stick: Movement
  LVel = max_speed*left.analog[1] + max_speed/2* left.analog[2];
  RVel = max_speed*left.analog[1] - max_speed/2* left.analog[2];
  LVel = math.min(max_speed,math.max(-max_speed*0.7,LVel));
  RVel = math.min(max_speed,math.max(-max_speed*0.7,RVel));

  LVel = LVel/2;
  RVel = RVel/2;

  hcm:set_control_wheel_velocity({LVel,RVel});

  --Right analog stick: Viewpoint control
  hcm:set_control_head_movement({-right.analog[2],right.analog[1]});

  -- Ignite toggle
  if button[5]==1 and old_button[5] == 0 then
    lidar_active = 1 - lidar_active;
    hcm:set_control_ignited(lidar_active);
  end

  old_button = button;

  control_data.control_lidar_active = lidar_active;
  control_data.control_head_movement = hcm:get_control_head_movement();
  control_data.control_wheel_velocity = hcm:get_control_wheel_velocity();
  control_data.control_ignited = hcm:get_control_ignited()[1];

  control_data.control_left_gripper = left.analog[3];
  control_data.control_right_gripper = right.analog[3];


  data = msgpack.pack(control_data);
  controller_channel:send(data);
		local ret = controller_channel_udp:send(data);

end

local maxFPS = 30
local inv_fps = 1/maxFPS
local cnt = 0
local t_start = unix.time()
local t_last = t_start
local t_debug = t_start
local t = t_last

while true do
  -- Timing
  t_last = t;
  t = unix.time()
  t_diff = t - t_last;
  if t_diff < inv_fps then
    unix.usleep( 1e6*(inv_fps-t_diff) )
    t_diff = inv_fps
  end
  t = unix.time()
  t_diff = t - t_last;
  update(t_diff);
  cnt = cnt+1
  -- Debugging: Once a second
  local t_diff_debug = t - t_debug
  if t_diff_debug > 0.2 then
    os.execute('clear')
    print( string.format('\nX360 | Updating at %.2f FPS, sending to %s port %d',
    cnt/t_diff_debug,Config.udp.UDP_IP	, udp_port) )
    print(string.format('Velocity: %d %d Lidar:%d IGNITED: %d', LVel, RVel, 
    lidar_active,  hcm:get_control_ignited()[1]) )
    t_debug = t;
    cnt = 0
  end
end

-- Exit
xbox360.close()
