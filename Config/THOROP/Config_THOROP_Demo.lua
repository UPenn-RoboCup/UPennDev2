assert(Config, 'Need a pre-existing Config table!')

-- Replace with a table of demo values
if not Config.demo then return Config end

local vector = require'vector'

local demo = {}
demo.waypoints = {
	['deans_reception'] = {
       vector.pose{1, 0, 0*DEG_TO_RAD},
        vector.pose{2, 0, 0*DEG_TO_RAD},
        vector.pose{3, 0, 0*DEG_TO_RAD},
        vector.pose{4, 0, 0*DEG_TO_RAD},
        vector.pose{5, 0, 0*DEG_TO_RAD},
--		vector.pose{1, 0, 0*DEG_TO_RAD},
--		vector.pose{1, 1, 90*DEG_TO_RAD},
--		vector.pose{2, 1, 0*DEG_TO_RAD},
--		vector.pose{10, 0, 0*DEG_TO_RAD},
	},
}
demo.arms = {
	['deans_ready'] = {
		--trLArm =
		--trRArm =
		qLArm = vector.new{101, 35, 0, -114, -208, 40, 175}*DEG_TO_RAD,
		qRArm = vector.new{70, -53, -36, -90, 60, 70, -11}*DEG_TO_RAD,
	},
}

Config.demo = demo

return Config
