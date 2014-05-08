-- Kalman filter; assume 60Hz refresh rate?
local libKalman = require'libKalman'
-- Assume only one libTouch2 per Lua process
local libTouch2 = {}

-- TODO: Add rpy of the tablet
local contacts = {}

local util = require'util'

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
	table.insert(k_to_c, false)
end

-- Update, knowing which was last updated
local function update_contact (c)
	-- Run the Kalman filter on touch c
	local k = c.kalman
	local observation = torch.DoubleTensor(2)
	observation[1] = c.x
	observation[2] = c.y
	-- See the number of frames to evolve
	local ndt = math.max(math.floor(c.dt * FPS), 1)
	k:predict():correct(observation)
	-- Update the contact information
	local x, p = k:get_state()
	c.x = x[1]
	c.y = x[2]
	c.vx = x[3]
	c.vy = x[4]
end

-- Start of a touch
local function start (o)
	print('start', o.id)
	assert(not contacts[o.id], "Contact exists!")
	-- Assign a Kalman filter if available
	local filter, kalman_id
	for i,k in ipairs(k_to_c) do
		if not k then
			filter = kalmans[i]
			kalman_id = i
			break
		end
	end
	assert(filter, "No filter available!")
	-- Reset to the position
	-- TODO: Reset noise
	filter.x_k_minus[1] = o.x
	filter.x_k_minus[2] = o.y
	filter.x_k_minus[3] = 0
	filter.x_k_minus[4] = 0
	filter.x_k:copy( filter.x_k_minus )

	-- Add to contacts
	local c = {
		x = o.x,
		y = o.y,
		vx = 0,
		vy = 0,
		t = o.t,
		e = 'start',
		id = o.id,
		kalman = filter,
		kalman_id = kalman_id,
	}
	-- Finsh the kalman lookup
	k_to_c[kalman_id] = c
	contacts[o.id] = c
	return c
end

-- Move of a touch
local function move (o)
	print('move', o.id)
	-- Find in the table
	local c = assert(contacts[o.id], "Not found for move!")
	-- Grab the time differential
	local dt = o.t - c.t
	-- Update the times
	c.t, c.dt = o.t, dt
	-- Update the positions
	local dx = o.x - c.x
	c.x, c.dx = o.x, dx
	local dy = o.y - c.y
	c.y, c.dy = o.y, dy
	-- General update
	update_contact (c)
	-- Save the event type
	c.e = 'move'
	return c
end

local function finish (o)
	print('finish', o.id)
	local c = assert(contacts[o.id], "Not found for finish!")
	-- Get the kalman data and reset it
	local k = c.kalman
	-- The filter is available now
	k_to_c[c.kalman_id] = false
	-- Remove from contacts
	contacts[o.id] = nil
	-- NOTE: The filter is still valid fro the callback
	c.e = 'finish'
	return c
end

local handlers = {
	start = start,
	move = move,
	stop = finish,
	leave = finish,
	cancel = finish,
}

-- Take in swipes from the network
-- Update each one in turn
local function update (swipes, cb)
	-- We get multiple swipes
	for i, swipe in ipairs(swipes) do
		-- Have a swipe, must process each entry
		-- Take the id from the first entry
		local id, c = swipe[1].id
		-- Look at each touch within the swipe
		for j, touch in ipairs(swipe) do
			-- Handle the event
			c = handlers[touch.e](touch)
			-- Run the callback for this contact
			if cb then cb(c) end
		end
		-- Done processing one swipe
	end
end

return {
	update = update,
}
