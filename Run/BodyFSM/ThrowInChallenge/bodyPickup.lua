module(..., package.seeall);

local Body = require('Body')
local Config = require('Config')
local Motion = require('Motion')
local walk = require('walk')
local wcm = require('wcm')
local grip = require 'grip' -- Grip and Throw engine

t0 = 0;
timeout = 20.0;
started = false;

--by SJ
pickable=true;

function entry()
	print(_NAME.." entry");

	t0 = Body.get_time();

	-- set kick depending on ball position

	local ball = wcm.get_ball();
	if( t0 - ball.t < .5 ) then -- Just saw it
		grip.set_distance( ball.x );
		print('Picking up ball (x,y):',ball.x, ball.y);
	end

	-- Start the pickup motion
	grip.throw = 0;	-- Do a pickup, not a throw
	Motion.event("pickup");
	started = false;
end

function update()
	local t = Body.get_time();

	if (not started and grip.active) then
		started = true;
	elseif (started and not grip.active) then
		return "done";
	end

	if (t - t0 > timeout) then
		return "timeout";
	end
end

function exit()
	-- Starting the walk should be part of the grip engine...?
--	walk.start();
end
