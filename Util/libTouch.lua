-- Kalman filter; assume 60Hz refresh rate?
local libKalman = require'libKalman'
-- Assume only one libTouch per Lua process
local libTouch = {}

-- TODO: Add rpy of the tablet
local contacts = {}
local trails = {}
-- Table of all gestures and their properties
-- TODO: Keep running table per session,
-- or prune once you send on the network
local props = {}

util = require'util'

local FPS = 60
local DECAY = .7

-- Initialize the Kalman filters:

local function generate_kalman()
	-- TODO: Tune these somehow
	local filter = libKalman.initialize_filter( 4 )
	-----------------
	-- Modify the Dynamics update
	-----------------
	filter.A:zero()
	-- Position stays the same
	filter.A:sub(1,2,  1,2):eye(2)
	-- TODO: blocks of the matrix may be mixed up...
	-- Predict next position by velocity
	filter.A:sub(1,2, 3,4):eye(2)
	-- Velocity Decay
	filter.A:sub(3,4, 3,4):eye(2):mul(DECAY)
	-----------------
	-- Modify the Measurement update
	-----------------
	-- We only measure the state positions, not velocities
	filter.R = torch.eye( 2 )
	filter.H = torch.Tensor( 2, 4 ):zero()
	filter.H:sub(1,2, 1,2):eye(2)
	--
	libKalman.initialize_temporary_variables( filter )
	return filter
end

-- Assume a max of five fingers...
-- TODO: Find easy way to map the changing
-- ids of the touches to the pool of filters
local kalmans, k_to_c, nKalman = {}, {}, 5
for i=1,nKalman do
	-- 2 dimensions (finger x and y)
	--local k = libKalman.new_filter(2)
	local k = generate_kalman()
	table.insert(kalmans,k)
	-- map to contacts (false is unused)
	table.insert(k_to_c,false)
end

-- Update, knowing which was last updated
local function update_contacts(id,c)
	-- Run the Kalman filter on touch c
	local k = c.kalman
	local observation = torch.DoubleTensor(2)
	observation[1] = c.x
	observation[2] = c.y
	-- See the number of frames to evolve
	local ndt = math.max(math.floor(c.dt * FPS),1)
	k:predict():correct( observation )
	return k
end

local function update_trails(id,c)
	-- Run the Kalman filter on touch c
	local k = c.kalman
	-- See the number of frames to evolve
	local ndt = math.max(math.floor(c.dt * FPS + 0.5),1)
	for i=1,ndt do
		k:predict()
	end
	return k
end

-- Handler API: timestamp, object
-- Refresh the system
libTouch.refresh = function(t,o)
	contacts = {}
	for i=1,nKalman do
		k_to_c[i] = false
	end
end

-- Start of a touch
libTouch.start = function(t,o)
	print('start',o.id)
	-- Assign a Kalman filter if available
	local filter, kalman_id
	for i,k in ipairs(k_to_c) do
		if not k then
			filter = kalmans[i]
			kalman_id = i
			break
		end
	end
	-- Finsh the kalman lookup
	if kalman_id then
		k_to_c[kalman_id] = contacts[o.id]
		-- Reset to the position
		filter.x_k_minus[1] = o.x
		filter.x_k_minus[2] = o.y
		filter.x_k_minus[3] = 0
		filter.x_k_minus[4] = 0
		filter.x_k:copy( filter.x_k_minus )
		-- TODO: Reset noise
	end
	-- Add a property
	table.insert(props,{
		move = {{x=o.x,y=o.y,vx=0,vy=0,t=t}},
		trail = {},
	})
	-- Add to contacts
	contacts[o.id] = {
		x = o.x,
		y = o.y,
		t = t,
		kalman = filter,
		kalman_id = kalman_id,
		prop_id = #props
	}
end

-- End of a touch
libTouch.stop = function(t,o)
	print('stop',o.id)
	-- Remove from table
	local c = contacts[o.id]
	if not c then
		print(o.id,'not found for stop')
		return
	end
	contacts[o.id] = nil
	trails[o.id] = c
	-- Final time when the mouse is up
	c.tf = t
	-- Still track the time
	local dt = t - c.t
	c.t = t
	-- Add a property
	props[c.prop_id].stop = {x=o.x,y=o.y,t=t}
	-- For continuity in the trail
	local x,P = c.kalman:get_state()
	table.insert(props[c.prop_id].trail,{
		x=x[1],y=x[2],vx=x[3],vy=x[4],
	})
end

-- Move of a touch
libTouch.move = function(t,o)
	print('move',o.id)
	-- Find in the table
	local c = contacts[o.id]
	if not c then
		print(o.id,'not found for move')
		return
	end
	-- TODO: Ensure that the id is present
	-- Grab the time differential
	local dt = t - c.t
	-- Update the times
	c.t, c.dt = t, dt
	-- Update the positions
	local dx = o.x - c.x
	c.x, c.dx = o.x, dx
	local dy = o.y - c.y
	c.y, c.dy = o.y, dy
	-- General update
	local k = update_contacts(o.id, c)
	-- Modify the property
	local x,P = k:get_state()
	util.ptorch(P)
	table.insert(props[c.prop_id].move,{
		x=x[1],y=x[2],vx=x[3],vy=x[4],
	})
end

libTouch.beat = function(t,o)
	print('beat',o.id)
	local c = contacts[o.id]
	if not c then
		print(o.id,'not found for beat')
		return
	end
	local dt = t - c.t
	c.t, c.dt = t, dt
	-- Add observation of no velocity?
	update_contacts(o.id, c)
	--
	local props = props[c.prop_id]
	local move_id = #props.move
	local beats = props.beats
	local cur_beat = beats and beats[#beats]
	if not cur_beat then
		-- Initial beat
		props.beats = {{i=move_id,n=1}}
	elseif cur_beat.i == move_id then
		cur_beat.n = cur_beat.n + 1
	else
		-- add new beat
		table.insert(props.beats,{i=move_id,n=1})
	end
end

libTouch.trail = function(t,o)
	print('trail',o.id,t)
	local c = trails[o.id]
	if not c then
		print(o.id,'not found for trail')
		return
	end
	local dt = t - c.t
	c.t, c.dt = t, dt
	local k = update_trails(o.id, c)
	local x,P = k:get_prior()
	util.ptorch(P)
	table.insert(props[c.prop_id].trail,{
		x=x[1],y=x[2],vx=x[3],vy=x[4],
	})
end

libTouch.finish = function(t,o)
	print('finish',o.id,t)
	local c = trails[o.id]
	if not c then
		print(o.id,'not found for finish')
		return
	end
	-- The filter is alailable now
	k_to_c[c.kalman_id] = false
	-- Return data to be sent to the browser
	return props[c.prop_id]
end

return libTouch
