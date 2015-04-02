assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

-- Replace with a table of demo values
if not Config.demo then return Config end

local demo = {}
demo.waypoints = {
	['deans_reception'] = {
		vector.pose{1, 0, 0*DEG_TO_RAD},
		vector.pose{1, 1, 90*DEG_TO_RAD},
		vector.pose{2, 1, 0*DEG_TO_RAD},
	},

}

return Config
