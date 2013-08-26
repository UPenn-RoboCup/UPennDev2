-- libWaypoint
-- (c) 2013 Stephen McGill
-- Perform Waypoint navigation, with a teleop overide

-- Timing
require 'unix'
local Body = require('Lib/ThorCentaurBody')

local libWaypoint = {}

local function set_waypoints( points )
	self.t_commanded = true
	self.mode = 'waypoint'
end

local function set_velocity( velocity )
	self.t_commanded = true
	self.mode = 'direct'
	self.vx = velocity.vx
	self.vy = velocity.vy
end

local function update_wheel_velocity ( self )
	local max_wheel_velocity = 4000
	local t_now = unix.time()
	local t_diff = t_now - self.last_update
	self.last_update = t_now
	
	-- Attenuate the velocity if not just commanded
	if self.mode == 'waypoint' then
		self:update_waypoint_velocity()
	else
		if self.t_commanded then
			self.t_commanded = false
		else
			self.vx = self.vx * self.attenuation*t_diff
			self.vy = self.vx * self.attenuation*t_diff
		end
	end

	-- TODO: better conversion factor
	Body.set_lwheel_velocity(self.vx)
	Body.set_rwheel_velocity(self.vy)
end

libWaypoint.new_follower = function()
	local follower = {}
	follower.set_waypoints = set_waypoints
	follower.set_velocity = set_velocity
	follower.update_wheel_velocity = update_wheel_velocity
	follower.vx = 0
	follower.vy = 0
	follower.attenuation = .9
end

return libWaypoint