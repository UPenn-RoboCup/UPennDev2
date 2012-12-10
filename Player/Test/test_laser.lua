-- CHARLI laser testing
print('Testing LIDAR on CHARLI')

cwd = cwd or os.getenv('PWD')
package.path = cwd.."/?.lua;"..package.path;
require('init')

require 'carray'

require('Config')
require('Body')
require('Speak')
require('Motion')
require('vector')

-- Laser getting
require 'WebotsLaser'
print( "LIDAR Dim:", WebotsLaser.get_width(), WebotsLaser.get_height())
nlidar_readings = WebotsLaser.get_width() * WebotsLaser.get_height();
require 'rcm'
--
require 'mcm'

-- Initialize Variables
webots = false;
teamID   = Config.game.teamNumber;
playerID = Config.game.playerID;
print '=====================';
print('Team '..teamID,'Player '..playerID)
print '=====================';
targetvel=vector.zeros(3);
if (string.find(Config.platform.name,'Webots')) then
  print('On webots!')
  webots = true;
end

-- Key Input
if( webots ) then
  controller.wb_robot_keyboard_enable( 100 );
else
  require 'getch'
  getch.enableblock(1);
end

-- Process Key Inputs
function process_keyinput()

  if( webots ) then
    str = controller.wb_robot_keyboard_get_key()
    byte = str;
    -- Webots only return captal letter number
    if byte>=65 and byte<=90 then
      byte = byte + 32;
    end
  else
    str  = getch.get();
    byte = string.byte(str,1);
  end
  --print('byte: ', byte)
  --print('string: ',string.char(byte))

  if byte==0 then
		return false
	end
	
	local update_walk_vel = false;
  -- Walk velocity setting
  if byte==string.byte("i") then	targetvel[1]=targetvel[1]+0.02; update_walk_vel = true;
  elseif byte==string.byte("j") then	targetvel[3]=targetvel[3]+0.1; update_walk_vel = true;
  elseif byte==string.byte("k") then	targetvel[1],targetvel[2],targetvel[3]=0,0,0; update_walk_vel = true;
  elseif byte==string.byte("l") then	targetvel[3]=targetvel[3]-0.1; update_walk_vel = true;
  elseif byte==string.byte(",") then	targetvel[1]=targetvel[1]-0.02; update_walk_vel = true;
  elseif byte==string.byte("h") then	targetvel[2]=targetvel[2]+0.02; update_walk_vel = true;
  elseif byte==string.byte(";") then	targetvel[2]=targetvel[2]-0.02; update_walk_vel = true;

  -- Walk commands
  elseif byte==string.byte("7") then
    Motion.event("sit");
  elseif byte==string.byte("8") then
    if walk.active then 
      walk.stopAlign();
    end
    Motion.event("standup");
  elseif byte==string.byte("9") then
    Motion.event("walk");
    walk.start();



  end
	
	if( update_walk_vel ) then
		print("Commanded velocity:",unpack(walk.velCommand))
		walk.set_velocity(unpack(targetvel));
	end
	
	return true
  
end

function update()
  count = count + 1;

  -- Update State Machines 
  Motion.update();
  Body.update();
	
	-- Update the laser scanner
	lidar_scan = WebotsLaser.get_scan()
	-- Set the Range Comm Manager values
	rcm.set_lidar_ranges( carray.pointer(lidar_scan) );
	rcm.set_lidar_timestamp( Body.get_time() )
	--print("Laser data:",unpack(rcm.get_lidar_ranges()))
	
	-- Show the odometry
	odom, odom0 = mcm.get_odometry();
	rcm.set_robot_odom( vector.new(odom) )
	
	-- Show IMU
  imuAngle = Body.get_sensor_imuAngle();
	rcm.set_robot_imu( vector.new(imuAngle) )
	gyr = Body.get_sensor_imuGyrRPY();
	rcm.set_robot_gyro( vector.new(gyr) );
	
  -- Check if the last update completed without errors
  lcount = lcount + 1;
  if (count ~= lcount) then
    print('count: '..count)
    print('lcount: '..lcount)
    Speak.talk('missed cycle');
    lcount = count;
  end
  io.stdout:flush();  
end

-- Initialize
Motion.entry()
count = 0;
lcount = 0;
tUpdate = unix.time();
Motion.event("standup");

-- if using Webots simulator just run update
local tDelay = 0.005 * 1E6; -- Loop every 5ms
while (true) do
	-- Run Updates
  process_keyinput();
  update();

  -- Debug Messages every 1 second
  t_diff = Body.get_time() - (t_last or 0);
  if(t_diff>1) then
		print('Odometry: ',odom)
		print('IMU: ',imuAngle)
		print('Gyro: ',vector.new(gyr) )
    t_last = Body.get_time();
  end

  if(darwin) then
    unix.usleep(tDelay);
  end
end
