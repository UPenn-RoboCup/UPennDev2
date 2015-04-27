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
	},
}

local smallbox = {
	--trLArm =
	--trRArm =
	qLArm = vector.new{106.287, 56.4949, -0.0387251, -113.479, -47.9627, -30.4474, 0.854519}*DEG_TO_RAD,
	qRArm = vector.new{76.7008, -42.5073, -32.2451, -89.8207, 48.285, 69.9745, -11.2936}*DEG_TO_RAD,
	qLGrip = vector.new{-12.3047, -49.1309, 6.76758}*DEG_TO_RAD,
	qRGrip = vector.new{-25.9668, -123.154, -60.9961}*DEG_TO_RAD,
}
local largebox = {
	--trLArm =
	--trRArm =
	qLArm = vector.new{99.4202, 46.1582, 0.027251, -82.8667, -44.5019, -69.7399, 18.4759}*DEG_TO_RAD,
	qRArm = vector.new{78.4585, -47.0316, -40.9898, -76.3207, 56.7093, 75.1538, -30.3858}*DEG_TO_RAD,
	qLGrip = vector.new{-32.5195, -20.127, 29.707}*DEG_TO_RAD,
	qRGrip = vector.new{-6.63086, -116.123, -48.3398}*DEG_TO_RAD,
}

demo.arms = {
	['dean'] = largebox
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

-- Left grip: 3rd is spport
-- Right grip: 2nd is support

-- Large box
--[[
>  =Body.get_rarm_command_position()*RAD_TO_DEG
[1] {78.4585, -47.0316, -40.9898, -76.3207, 56.7093, 75.1538, -30.3858}
>  =Body.get_larm_command_position()*RAD_TO_DEG
[1] {99.4202, 46.1582, 0.027251, -82.8667, -44.5019, -69.7399, 18.4759}
>  =Body.get_rgrip_position()*RAD_TO_DEG
[1] {-6.63086, -116.123, -48.3398}
>  =Body.get_lgrip_position()*RAD_TO_DEG
[1] {-32.5195, -20.127, 29.707}
--]]


Config.demo = demo

return Config
