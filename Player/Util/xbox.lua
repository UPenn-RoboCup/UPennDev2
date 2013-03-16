module(... or "",package.seeall)
require 'unix'
require 'xbox360'
require 'Config'
require 'util'

fps = 60; -- Same sampling as the primesense

-- Scale factors
joy_resolution = 32768;
joy_deadband = joy_resolution/8;
maxStep = 0.04;
scale_vx = maxStep/joy_resolution;
scale_vy = -1*maxStep/joy_resolution;
maxA = 0.1;
scale_va = -1*maxA/joy_resolution;

-- Arm and torso shifts
--maxdArm = 0.005;
maxdArm = 0.01;
--maxdArm = 0.025;
--maxdArm = 1e-5;
scale_dpX = maxdArm / joy_resolution;
scale_dpY = -1 * maxdArm / joy_resolution;
scale_dbh = maxdArm / joy_resolution;

--[[
Buttons
1: D-Pad up	 D-Pad up
2: D-Pad down	 D-Pad down
3: D-Pad left	 D-Pad left
4: D-pad right	 D-Pad right
5: Start button	 Button 8
6: Back button	 Button 7
7: Left stick press	 Button 9
8: Right stick press	 Button 10
9: Button LB	 Button 5
10: Button RB	 Button 6
11: Xbox logo button
12: Unused
13: Button A	 Button 1
14: Button B	 Button 2
15: Button X	 Button 3
16: Button Y	 Button 4

Axes
1	 Left trigger	 Z-axis down
2	 Right trigger	 Z-axis up
3	 Left stick X-axis	 X-axis
4	 Left stick Y-axis	 Y-axis
5	 Right stick X-axis	 X-turn
6	 Right stick Y-axis	 Y-turn
7	 Unused
--]]

function entry()
	xbox360.open()
	timestamp0 = unix.time();
end

function update()
	count = count+1
	buttons = xbox360.button()
	axes = xbox360.axis()
	--print(unpack(axes))
	--unix.usleep(1e6/fps)
	return true;
end

function exit()
	xbox360.close()
end

function get_velocity()
	-- Left stick Y-axis controls vx
	local vx = axes[4]
	if math.abs(vx)<joy_deadband then
		vx = 0;
	end
	vx = scale_vx*vx;
	
	-- Left stick X-axis controls vy
	local vy = axes[3]
	if math.abs(vy)<joy_deadband then
		vy = 0;
	end
	vy = scale_vy*vy;

	-- Right stick X-axis controls va
	local va = axes[5]
	if math.abs(va)<joy_deadband then
		va = 0;
	end
	va = scale_va*va;

	local velocity = vector.new({vx,vy,va});
	return velocity;
end

function get_dpLArm( )
	-- Left stick Y-axis controls dx
	local dpX = axes[4]
	if math.abs(dpX)<joy_deadband then
		dpX = 0;
	end
	dpX = scale_dpX * dpX;

	-- Left stick X-axis controls dy
	local dpY = axes[3]
	if math.abs(dpY)<joy_deadband then
		dpY = 0;
	end
	dpY = scale_dpY * dpY;

  if buttons[7]==1 then
    dpZ = dpX
    dpX = 0;
    dpY = 0
  else
    dpZ = 0;
  end

  return vector.new({dpX,dpY,dpZ});	
end

function get_dpRArm( )
	-- Left stick Y-axis controls dx
	local dpX = axes[6]
	if math.abs(dpX)<joy_deadband then
		dpX = 0;
	end
	dpX = scale_dpX * dpX;

	-- Left stick X-axis controls dy
	local dpY = axes[5]
	if math.abs(dpY)<joy_deadband then
		dpY = 0;
	end
	dpY = scale_dpY * dpY;

  if buttons[8]==1 then
    dpZ = dpX
    dpX = 0;
    dpY = 0
  else
    dpZ = 0;
  end
  
  return vector.new({dpX,dpY,dpZ});	
end

function get_rpy()
  local rpy = vector.new({0,-1*Config.walk.bodyTilt,0});
  if buttons[13]==1 then
    rpy = vector.new({0,20,0})*math.pi/180;
  end
  return rpy;
end

function get_dbodyHeight( )
	-- Use the dpad
	local direction = 0
	if buttons[1]==1 then  --up
		direction = 1
	elseif buttons[2]==1 then --down
		direction = -1
	end
	local dbodyHeight = scale_dbh * direction
--	local bH = math.max(math.min(bodyHeight0+dbodyHeight,Config.walk.bodyHeight),Config.stance.bodyHeightSit)
	return dbodyHeight;
end

