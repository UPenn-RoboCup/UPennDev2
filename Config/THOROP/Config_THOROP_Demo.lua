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
		qLArm = vector.new{106.287, 56.4949, -0.0387251, -113.479, -47.9627, -30.4474, 0.854519}*DEG_TO_RAD,
		qRArm = vector.new{76.7008, -42.5073, -32.2451, -89.8207, 48.285, 69.9745, -11.2936}*DEG_TO_RAD,
		qLGrip = vector.new{-12.3047, -49.1309, 6.76758}*DEG_TO_RAD,
		qRGrip = vector.new{-25.9668, -123.154, -60.9961}*DEG_TO_RAD
	},
}

--[[

>  =Body.get_larm_position()*RAD_TO_DEG
[1] {106.289, 56.4949, -0.0372908, -113.477, -47.9627, -30.4474, 0.854519}
>  =Body.get_rarm_position()*RAD_TO_DEG
[1] {76.7001, -42.5087, -32.2444, -89.82, 48.2874, 69.9721, -11.296}
>  return Body.get_lgrip_position()*RAD_TO_DEG
[1] {-12.3047, -49.1309, 6.76758}
>  return Body.get_rgrip_position()*RAD_TO_DEG
[1] {-25.9668, -123.154, -60.9961}

--]]
Config.demo = demo

return Config
