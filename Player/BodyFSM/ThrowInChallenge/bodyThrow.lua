module(..., package.seeall);

require('Body')
require('Config')
require('Motion')
require('wcm')
require 'grip' -- Grip and Throw engine

t0 = 0;
timeout = 15.0;
started = false;

function entry()
	print(_NAME.." entry");

	t0 = Body.get_time();
	ball = wcm.get_ball();

	-- Start the throwing motion
	grip.throw = 1;
	Motion.event("throw");
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
end

