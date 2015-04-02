assert(Config, 'Need a pre-existing Config table!')

-- Replace with a table of demo values
if not Config.demo then return Config end

local vector = require'vector'

local dean_reception_waypoints = {
	vector.pose{1, 0, 0*DEG_TO_RAD},
	vector.pose{1, 1, 90*DEG_TO_RAD},
	vector.pose{2, 1, 0*DEG_TO_RAD},
}

Config.demo = {
	waypoints = dean_reception_waypoints
}

return Config
