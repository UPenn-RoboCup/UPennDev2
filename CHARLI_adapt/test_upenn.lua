dofile('include.lua');
package.path = "./upenn/?.lua;"..package.path;
require 'rcm'
require 'Sensor'
odom = vector.zeros(3)


require('Config');
require('Robot');
require('Head');
require('Walk');
require('Action');
require('vcm')
require('unix')
require('vector')
require('getch')
require('Speak2')

getch.enableblock(1);

-- initialize robot
Robot.entry();
vcm.shmInit();

-- control variables
tDelay = 4*1e3
head_angle = {0.0, 0.0};
walk_velocity = {0.0, 0.0, 0.0};
head_set = false;
walk_set = false;
walk_enable = false;
current_action = 'walk_ready'

-- action / audio config
action_keys = Config.demo.action_keys
audio_enable = Config.demo.audio_enable




function printf(s,...)
	return io.write(s:format(...))
end

function draw_status()
  unix.system('clear')
  printf(' _\n')
  printf('|_) _ |V| _ |   _\n');
  printf('| \\(_)| |(/_|__(_|\n');
  printf('\n');
  printf('walk enable    :  %s\n', tostring(walk_enable));
  printf('walk velocity  : %5.2f, %5.2f, %5.2f\n', walk_velocity[1], walk_velocity[2], walk_velocity[3]);
  printf('head angles    : %5.2f, %5.2f\n', unpack(head_angle));
  printf('current action :  %s\n', current_action);
  printf('LIDAR counter  : %d\n',rcm.get_lidar_counter())
  printf('Odom           : %f,%f,%f',unpack(odom))
  printf('\n');
  printf('Hit / for help ...\n');
end

function draw_help()
  unix.system('clear')
  printf(' _\n');
  printf('|_) _ |V| _ |   _\n');
  printf('| \\(_)| |(/_|__(_|\n');
  printf('\n');
  printf('walk enable    : space\n');
  printf('walk motion    : i j k l ,\n');
  printf('head motion    : e s d f c\n');
  printf('action brake   : `\n');
  for k,v in pairs(action_keys) do
     printf('%-14s : %s\n', v[1], k); 
  end
  printf('\n');
  printf('Hit <enter> to continue ...\n');
end

-- handler for key commands
function cmd_process(str)

	local byte=string.byte(str,1);

	-- Head controls
	if byte==string.byte("w") then
		head_angle[2] = head_angle[2]-3;
		head_set = true;
	elseif byte==string.byte("a") then
		head_angle[1] = head_angle[1]+3;
		head_set = true;
	elseif byte==string.byte("s") then
		head_angle[1],head_angle[2] = 0,0;
		head_set = true;
	elseif byte==string.byte("d") then
		head_angle[1] = head_angle[1]-3;
		head_set = true;
	elseif byte==string.byte("x") then
		head_angle[2] = head_angle[2]+3;
		head_set = true;
	elseif byte==string.byte("q") then
		head_angle[1] = head_angle[1]+3;
		head_angle[2] = head_angle[2]-3;
		head_set = true;
	elseif byte==string.byte("e") then
		head_angle[1] = head_angle[1]-3;
		head_angle[2] = head_angle[2]-3;
		head_set = true;
	elseif byte==string.byte("z") then
		head_angle[1] = head_angle[1]+3;
		head_angle[2] = head_angle[2]+3;
		head_set = true;
	elseif byte==string.byte("c") then
		head_angle[1] = head_angle[1]-3;
		head_angle[2] = head_angle[2]+3;
		head_set = true;

	-- Walk controls
	elseif byte==string.byte("i") then	
		walk_velocity[1] = walk_velocity[1]+0.01;
		walk_set = true;
	elseif byte==string.byte("h") then
		walk_velocity[2] = walk_velocity[2]+0.01;
		walk_set = true;
	elseif byte==string.byte("j") then
		walk_velocity[3] = walk_velocity[3]+1;
		walk_set = true;
	elseif byte==string.byte("k") then
		walk_velocity[1],walk_velocity[2],walk_velocity[3] = 0,0,0;
		walk_set = true;
	elseif byte==string.byte("l") then
		walk_velocity[3] = walk_velocity[3]-1;
		walk_set = true;
	elseif byte==string.byte(";") then
		walk_velocity[2] = walk_velocity[2]-0.01;
		walk_set = true;
	elseif byte==string.byte(",") then
		walk_velocity[1] = walk_velocity[1]-0.01;
		walk_set = true;
	elseif byte==string.byte(" ") then
		walk_enable = not walk_enable;
		if(walk_enable) then
			Action.brake();
			Action.start('walk_ready');
		        Head.set_angle(0, 0)
			while(Action.is_running()) do end;
			Walk.set_velocity(unpack(walk_velocity));
			Walk.start();
		else 
			Walk.stop();
		end

	-- Special Action controls 
	elseif byte==string.byte("`") then
		Action.brake();
		current_action = '';
	end

	-- Standard Action Controls
	for k,v in pairs(action_keys) do
		if byte==string.byte(k) then
			current_action = v[1] or '';
			if v[1] then
				Action.start_upperbody(v[1]);
			end
			if audio_enable then
				Speak2.play(v[2] or '/dev/null');
			end
		end
	end
end

-- goto initial pose
draw_status()
Action.start('walk_ready');
while(Action.is_running()) do end;

-- main loop
count = 0;
loop = true;
t = unix.time();
while(loop) do

--print(rcm.get_lidar_counter(),rcm.get_lidar_timestamp() )

	t0 = t;
	t = unix.time();
	count = count + 1;

	-- Wait
	unix.usleep(tDelay);

	-- Update modules 
	Robot.update();

	-- NEW
rcm.set_robot_gyro( vector.new( {Sensor.imu_gyro()} ) )
rcm.set_robot_imu( vector.new( {Sensor.imu_angle()} ) )
odom[1] = odom[1] + walk_velocity[1]*(t-t0) * Config.walk.velocity_factor[1];
odom[2] = odom[2] + walk_velocity[2]*(t-t0) * Config.walk.velocity_factor[2];
odom[3] = 0;
rcm.set_robot_odom( odom );
	---

	-- Process keypress
	local str = getch.get();
	if #str>0 then 
		if string.byte(str,1)==string.byte("/") then
			draw_help();
		else
			cmd_process(str); 
			draw_status();
		end
	end

	-- Update head
	if(Action.is_running()) then
		head_angle[1], head_angle[2] = Head.get_angle()
		Head.set_angle(Head.get_angle())
	end
	if(head_set) then
		head_set = false;
		Head.set_angle(unpack(head_angle));
	end

	-- Update walk
	if(walk_set) then
		walk_set = false;
		Walk.set_velocity(unpack(walk_velocity));
	end
end
