dofile './include.lua'
dofile '../RunPenn/networkparam.lua'

local jcm = require'jcm'
local hcm = require'hcm' --HID shm
local msgpack = require 'msgpack'
require 'unix'

local Transform = require ('Lib/TransformPenn');
local Body = require('Lib/ThorCentaurBody') --For angle conversion
--local hid = require ('Lib/hid') --Hydra driver
local hid = require ('hid') --Hydra driver

-- 0x1532 0x0300  razer hydra
ud = hid.init(0x1532, 0x0300)
ud:stream_on();
init_done = false;

left_button_old = {}
right_button_old = {}

left_pos_f={};
right_pos_f={};

left_pos0={};
right_pos0={};

local left_mode = 'None'
local right_mode = 'None'


local udp = require 'udp'
controller_channel_udp = udp.new_sender(UDP_IP, PORT_CONTROL);
local t_last_udp = unix.time()


function send_udp()
  control_data={};
  control_data.control_wheel_velocity = hcm:get_control_wheel_velocity();
  control_data.control_head_movement = hcm:get_control_head_movement();
  control_data.control_lidar_active = hcm:get_control_lidar_active()[1];
  control_data.control_left_arm_movement = hcm:get_control_left_arm_movement();
  control_data.control_right_arm_movement = hcm:get_control_right_arm_movement();
  control_data.control_left_arm_mode = hcm:get_control_left_arm_mode()[1];
  control_data.control_right_arm_mode = hcm:get_control_right_arm_mode()[1];
  control_data.control_left_gripper = hcm:get_control_left_gripper()[1];
  control_data.control_right_gripper = hcm:get_control_right_gripper()[1];
  control_data.control_emergency_stop = hcm:get_control_emergency_stop()[1];

  data = msgpack.pack(control_data);
  controller_channel_udp:send(data);
  local ret = controller_channel_udp:send(data);
end

function e_filter(val_f_old, val)
  local filter_threshold = 5;
  local gamma = 0.5;
  if math.abs(val_f_old-val)<filter_threshold then
    return val_f_old;
  else
    local val_new = (1-gamma)*val_f_old + gamma*val;
    return val_new;
  end
end

function process_hydra(count)
  tbl= ud:get()
  if (type(tbl) == 'table') then

    --L/R fixing 
    left = tbl.left;
    right = tbl.right;

    left_pos={};
    left_pos[1] = left.pos[2]; 
    left_pos[2] = left.pos[1];
    left_pos[3] = -left.pos[3];
    if (left_pos[3]<0) then 
      left_pos[1] = -left_pos[1];
      left_pos[2] = -left_pos[2];
      left_pos[3] = -left_pos[3];
    end
    right_pos={};
    right_pos[1] = right.pos[2]; 
    right_pos[2] = right.pos[1];
    right_pos[3] = -right.pos[3];
    if (right_pos[3]<0) then 
      right_pos[1] = -right_pos[1];
      right_pos[2] = -right_pos[2];
      right_pos[3] = -right_pos[3];
    end

    if not init_done then
      left_button_old = left.button;
      right_button_old = right.button;
      left_pos_f = {left_pos[1],left_pos[2],left_pos[3]};
      right_pos_f = {right_pos[1],right_pos[2],right_pos[3]};
      init_done =  true;
    end

    left_button = left.button;
    right_button = right.button;

    --Updated filtered position 
    left_pos_f[1] = e_filter(left_pos_f[1],left_pos[1]);
    left_pos_f[2] = e_filter(left_pos_f[2],left_pos[2]);
    left_pos_f[3] = e_filter(left_pos_f[3],left_pos[3]);
    right_pos_f[1] = e_filter(right_pos_f[1],right_pos[1]);
    right_pos_f[2] = e_filter(right_pos_f[2],right_pos[2]);
    right_pos_f[3] = e_filter(right_pos_f[3],right_pos[3]);

    hcm:set_control_left_arm_movement(left_pos_f);
    hcm:set_control_right_arm_movement(right_pos_f);

    --E-stop: hit both bottom button at once
    if left_button[6]==1 and right_button[6]==1 then 
       print('Emergency Stop\n')
       hcm:set_control_emergency_stop(1);
    else
       hcm:set_control_emergency_stop(0);
    end

    --Left control 
    --Back button pushed: Arm movement 
    if left_button[1]==1 then 
	hcm:set_control_left_arm_movement(left_movement)
        if left_button[2] ==1 then --Button 1
          hcm:set_control_left_arm_mode(1);  --Initiate arm sequence
	  left_mode = 'Init Arm Sequence'
        elseif left_button[3] ==1 then  --Button 2
          hcm:set_control_left_arm_mode(2);  --Reset arm sequence
	  left_mode = 'Reset Arm Sequence'
        else --No button
          hcm:set_control_left_arm_mode(9); --Tele-operation mode

	  --Left analog stick: wrist rotation
	  hcm.set_control_left_wrist_rotation(left.analog);
	  left_mode = 'Teleoperation'
        end
    else     
    --Back button not pushed: Other commands
       hcm:set_control_left_arm_mode(0); --STOP if the movement button is released
       left_mode = 'Stop'
       hcm:set_control_left_arm_movement({0,0,0});     


       max_speed =9000;

      --Left analog stick: Movement
       LVel = max_speed*left.analog[1] + max_speed/2* left.analog[2];
       RVel = max_speed*left.analog[1] - max_speed/2* left.analog[2];
       LVel = math.min(max_speed,math.max(-max_speed*0.7,LVel));
       RVel = math.min(max_speed,math.max(-max_speed*0.7,RVel));
       hcm:set_control_wheel_velocity({LVel,RVel});

       if left_button[2] ==1 and left_button_old[2] == 0 then 
	  --Button 1 pressed: Toggle Logging?
       end
       if left_button[2] ==1 then 

       end
    end

    --Right control 

    --Back button pushed: Arm movement 
    if right_button[1]==1 then 
	hcm:set_control_right_arm_movement(right_movement)
        if right_button[2] ==1 then 
          hcm:set_control_right_arm_mode(1); --Initiate arm sequence
	  right_mode = 'Init Arm Sequence'
        elseif right_button[3] ==1 then 
          hcm:set_control_right_arm_mode(2); --Reset arm sequence
	  right_mode = 'Reset Arm Sequence'
        else
          hcm:set_control_right_arm_mode(9); --Tele-operation mode
	  --Right analog stick: wrist rotation
	  hcm.set_control_right_wrist_rotation(right.analog);
	  right_mode = 'Teleoperation'
        end
    else   
    --Back button not pushed: Other commands
       hcm:set_control_right_arm_mode(0); --STOP if the movement button is released
       right_mode = 'Stop'
       hcm:set_control_right_arm_movement({0,0,0});    

       --Right analog stick: Viewpoint control
       hcm:set_control_head_movement({-right.analog[2],right.analog[1]});
    end

    --Analog trigger: gripper control
    hcm:set_control_left_gripper({left.analog[3]});
    hcm:set_control_right_gripper({right.analog[3]});

    --thumb button: angle control
    --printf(left.rpy[1], left.rpy[2], left.rpy[3], right.rpy[1], right.rpy[2], right.rpy[3])

    left_button_old = left.button;
    right_button_old = right.button;


    return true;
  end
  return false;
end





local t0 = unix.time();
local count = 0;
local t_message = 0.10; --10fps message rate


while true do
  local t = unix.time();
  received = process_hydra(count);  
  unix.usleep(1E6/250); --Default control rate is ~250hz

  if count%10==0 then --25fps udp send rate
    send_udp();  
  end
  count = count + 1;

  if t-t0>t_message then
    os.execute('clear');
    print( string.format('\nHydra | Updating at %.2f FPS, sending to %s port %d',
	count/(t-t0),UDP_IP ,PORT_CONTROL) );

    print( string.format('\nLPos: %d %d %d | RPos: %d %d %d\n',
       left_pos_f[1],
       left_pos_f[2],
       left_pos_f[3],
       right_pos_f[1],
       right_pos_f[2],
       right_pos_f[3] ));

    print( string.format('Left Mode: %s | Right Mode: %s\n',
       left_mode,
       right_mode ));

    count=0;
    t0=t;
  end


end
