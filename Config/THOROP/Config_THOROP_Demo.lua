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
	['dean'] = {
		--trLArm =
		--trRArm =
		qLArm = vector.new{101, 35.0032, 0.00215139, -113.999, -202.973, 39.9964, 174.995}*DEG_TO_RAD,
		qRArm = vector.new{70, -53, -36, -90, 60, 70, -11}*DEG_TO_RAD,
		qLGrip = vector.new{-6, -50, 13}*DEG_TO_RAD,
		qRGrip = vector.new{5, -66, -64}*DEG_TO_RAD
	},
}

--[[

>  return Body.get_larm_position()*RAD_TO_DEG
[1] {100.157, 23.0945, -14.154, -71.2635, -37.0116, -70.9179, 38.688}
>  return Body.get_rarm_position()*RAD_TO_DEG
[1] {85.4061, -26.2384, -20.0216, -60.7991, 207.627, -76.7064, -37.1212}
>  return Body.get_lgrip_position()*RAD_TO_DEG
[1] {-5.97656, -50.4492, 13.2715}
>  return Body.get_rgrip_position()*RAD_TO_DEG
[1] {-76.2402, -27.5293, -39.2871}

--]]
Config.demo = demo

return Config
