-- Kalman filter; assume 60Hz refresh rate?
local libKalman = require'libKalman'
-- Assume only one libTouch per Lua process
local libTouch = {}

-- TODO: Add rpy of the tablet
local contacts = {}
local trails = {}

-- Initialize the Kalman filters:

local function generate_kalman()
	-- TODO: Tune these somehow
	local DECAY = .8
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
	-- NOTE: c.dt is important... if finger in contact but not moving
	local x,P = k:predict():correct( observation ):get_state()
	--print('\nraw',c.x,c.y,1/c.dt)
	--print('filt',x[1],x[2],x[3],x[4])
	print('Vel',x[3],x[4])
	print(x[1],x[2])
end

local function update_trails(id,c)
	-- Run the Kalman filter on touch c
	local k = c.kalman
	-- NOTE: c.dt is important...
	local x,P = k:predict():get_prior()
	print('filt',x[1],x[2],x[3],x[4])
end

-- Handler API: timestamp, object
-- Refresh the system
libTouch.refresh = function(t,o)
	contacts = {}
	-- TODO: Reset the Kalman filters
	for i=1,nKalman do
		k_to_c[i] = false
		-- TODO: kalmans[i]:reset()
	end
end

-- Start of a touch
libTouch.start = function(t,o)
	-- Assign a Kalman filter if available
	local kalman, kalman_id
	for i,k in ipairs(k_to_c) do
		if not k then
			kalman = kalmans[i]
			kalman_id = i
			break
		end
	end
	-- Add to contacts
	contacts[o.id] = {
		x0 = o.x,
		y0 = o.y,
		t0 = t,
		kalman = kalman,
		kalman_id = kalman_id,
	}
	-- Finsh the kalman lookup
	if kalman_id then k_to_c[kalman_id] = contacts[o.id] end
end

-- End of a touch
libTouch.stop = function(t,o)
	-- Remove from table
	local c = contacts[o.id]
	if not c then
		print(o.id,'not found for stop')
		return
	end
	contacts[o.id] = nil
	trails[o.id] = c
end

-- Move of a touch
libTouch.move = function(t,o)
	-- Find in the table
	local c = contacts[o.id]
	if not c then
		print(o.id,'not found for move')
		return
	end
	-- TODO: Ensure that the id is present
	-- Grab the time differential
	local dt = t - (c.t or c.t0)
	-- Update the times
	c.t, c.dt = t, dt
	-- Update the positions
	local dx = o.x - (c.x or c.x0)
	c.x, c.dx = o.x, dx
	local dy = o.y - (c.y or c.y0)
	c.y, c.dy = o.y, dy
	-- General update
	update_contacts(o.id, c)
end

libTouch.beat = function(t,o)
	print('heartbeat',o.id)
end

libTouch.trail = function(t,o)
	print('trail',o.id,t)
	local c = trails[o.id]
	if not c then
		print(o.id,'not found for trail')
		return
	end
	update_trails(o.id, c)
end

libTouch.finish = function(t,o)
	print('finish',o.id,t)
	local c = trails[o.id]
	if not c then
		print(o.id,'not found for finish')
		return
	end
	-- Reset the relevant Kalman filter
	k_to_c[c.kalman_id] = false
	-- TODO: c.kalman:reset()
end

return libTouch
