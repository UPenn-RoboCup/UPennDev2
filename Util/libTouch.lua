-- Kalman filter; assume 60Hz refresh rate?
local libKalman = require'libKalman'
-- Assume only one libTouch per Lua process
local libTouch = {}

-- TODO: Add rpy of the tablet
local contacts = {}

-- Initialize the Kalman filters:
-- Assume a max of five fingers...
-- TODO: Find easy way to map the changing
-- ids of the touches to the pool of filters
local kalmans, k_to_c, nKalman = {}, {}, 5
local k_to_c
for i=1,nKalman do
	-- 2 dimensions (finger x and y)
	local k = libKalman.new_filter(2)
	table.insert(kalmans,k)
	-- map to contacts (false is unused)
	table.insert(k_to_c,false)
end

-- Update, knowing which was last updated
local function update(id,c)
	-- Run the Kalman filter on touch c
	local k = c.kalman
	local x,P = k:predict():correct( obs1 ):get_state()
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
libTouch.end = function(t,o)
	-- Remove from table
	local c = contacts[o.id]
	contacts[o.id] = nil
	-- Reset the relevant Kalman filter
	k_to_c[c.kalman_id] = false
	-- TODO: c.kalman:reset()
end

-- Move of a touch
libTouch.move = function(t,o)
	-- Find in the table
	local c = contacts[o.id]
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
	update(o.id, c)
end

return libTouch
