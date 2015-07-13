assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

Config.enable_log = false
Config.enable_log = true

local monitor = {
	minFPS = 1,
	maxFPS = 15,
}

-- TODO: put this into FSM config, and state-specific
local head = {
	pitchMin = -10 * DEG_TO_RAD,
	pitchMax = 75 * DEG_TO_RAD,
	yawMin = -135 * DEG_TO_RAD,
	yawMax = 135 * DEG_TO_RAD,
	-- Head angle bias
	pitchBias = 0,
	yawBias = 0,
	-- From CoM to neck joint: {neckX, neckY, neckZ}
	neckOffset = {0, 0, 0.285}
}

local colors = {
	black = 0,
	orange = 1,
	yellow = 2,
	blue = 4,
	field = 8,
	white = 16,
	cyan = 32,
	magenta = 64,
}

local vision = {
	colors = colors,
	scaleA = 2,
	scaleB = 2,
}

local vision_k2 = {
	colors = colors,
	scaleA = 1,
	scaleB = 2,
}

vision.ball = {
	diameter = 0.22,
	th_min_bbox_area = 40, --50
	th_min_area = 20, --10,
	th_min_fill_rate = 0.35,

	--TODO: to test on real robot
	max_height0 = 0.3,    --Max height = max_height0 + dist*max_height1
	max_height1 = 0.12,

	max_distance = 9,
	th_ground_head_pitch = 50*DEG_TO_RAD,
	th_ground_boundingbox = {-30,30,0,20},
	th_ground_green = 400,  --TODO
	th_ground_white = 150,  --TODO
	check_for_ground = 1,
	check_for_field = 1,
	field_margin = 2.0,
}

vision.obstacle = {
	label = 'b',
	grid_x = 40, --32,
	grid_y = 20, --18,
	th_min_area = 40,
	min_black_fill_rate = 0.48,
	th_aspect_ratio = {1.8, 10},
	th_max_height = 1.3,
	th_min_height = -0.2,
	th_min_orientation = 60/180*math.pi,
	th_green_black_ratio = 2,
	min_ground_fill_rate = 0.4,
	--
	min_width = 5,
	max_width = 16,
}

vision.goal = {
	th_min_bbox_area = 80, --100,
	th_nPostB = 15,
	th_min_area = 40,
	th_min_orientation = 80*DEG_TO_RAD,
	th_min_fill_rate = 0.4, --0.45,
	-- TODO: need to test on real robot
	height_min = 1,
	height_max = 2,
	th_aspect_ratio = {13,120},
	th_edge_margin = 5,
	th_bottom_boundingbox = 0.9,
	th_ground_boundingbox = {-15,15,-15,10},
	th_min_green_ratio = 0.2,
	th_min_bad_color_ratio = 0.1,
	th_goal_separation = {0.35,3.0},
	th_min_area_unknown_post = 80,
	-- The range we triangulate:
	far_goal_threshold= 4.0,
	use_centerpost = 1,
	min_crossbar_ratio = 0.6,
	check_for_ground = 1,
	-- Metric properties
	goalHeight = 1.8,
	goalWidth = 3.1, --3.0 for I-I, 3.1 for C-C
	postDiameter = 0.1,
}

vision.line = {
  -- min_white_pixel = 300,
  -- min_green_pixel = 5000,
  --max_width = 15,
	max_width = 5,
  --connect_th = 1.4,
	connect_th = 3,
  --max_gap = 1,
	max_gap = 2,
  -- labelB space
  min_count = 20,
  --min_length = 5,
	min_length = 8,
  --max_height = 0.3,
	max_height = 0.2,
  --min_aspect_ratio = 2.5,
	min_aspect_ratio = 3,
  min_angle_diff = 10,
  max_angle_diff = 85,
}

local grasp920_param = {
	{'White Balance Temperature', 2400},
	{'Exposure (Absolute)', 120},
	--	{'Focus (absolute)', 0},
	{'Brightness', 128},
	{'Contrast', 0},
	{'Saturation', 255},
	{'Gain', 0},
	{'Sharpness', 0},
}
local grasp900_param = {
	{'White Balance Temperature', 0},
	{'Exposure (Absolute)', 170},
	--{'Focus', 0},
	{'Brightness', 0},
	{'Contrast', 8},
	{'Saturation', 16},
	{'Gain', 0},
	{'Sharpness', 0},
}

local ucla_param = {
	{'White Balance Temperature', 3350},
	{'Exposure (Absolute)', 120},
	--	{'Focus (absolute)', 0},
	{'Brightness', 128},
	{'Contrast', 128},
	{'Saturation', 173},
	{'Gain', 0},
	{'Sharpness', 0},
}
-- Cameras
local camera = {}
camera[1] = {
	name = 'head',
	--dev = '/dev/video-headcamera',
	dev = '/dev/video1',
	w = 640,
	h = 360,
	--dev = '/dev/video-wristcamera',
	--w = 640,
	--h = 480,
	format = 'yuyv',
	fps = 30,
	quality = 80,
	downsampling = 1,
	--crop = {640/2+1, 360/2+1, 640/2, 360/2},

	-- cameraRoll, cameraPitch, cameraYaw
	mountOffset = {
		--[[
		{0*DEG_TO_RAD,-5*DEG_TO_RAD,0},
		{0.02,0,0.14}
		--]]
		{0*DEG_TO_RAD,0*DEG_TO_RAD,0},
		{0.05,0,0.14}

	},
	cx_offset = 0,
	cy_offset = 0,
	-- Use the standard head properties
	head = head,
	-- Use the default vision parameters
	vision = vision,
	-- Run the standard RoboCup vision processing
	lut = 'test',
	--Logitech C920
	-- f = 640/2/tan(78/180*pi / 2)
	--fov = 2*arctan(d/2f)
	-- f = d/2/tan(fov/2)
	--webots fov: 1.3613
	focal_length = 395.17,
	focal_base = 640,
	auto_param = {
		{'Exposure, Auto', 0},
		{'White Balance Temperature, Auto', 0},
		{'Power Line Frequency', 0},
		{'Exposure, Auto Priority', 1},
		{'Focus, Auto', 0}
	},
	param = grasp920_param,
	--param = ucla_param,
}

camera[2] = {
	name = 'wrist',
	dev = '/dev/video-wristcamera',
	format = 'yuyv',
	w = 320,
	h = 240,
	fps = 5,
	jpeg_quality = 50,
	quality = 50,
	downsampling = 1,
	crop = {320/2+1, 240/2+1, 320/2, 240/2},
	auto_param = {
		{'Exposure, Auto', 1},
		{'White Balance Temperature, Auto', 1},
		{'Power Line Frequency', 0},
		{'Exposure, Auto Priority', 1},
		--{'Focus, Auto', 0}
	},
	param = m308_param,
}

local kinect = {
	name = 'kinect2',
	mountOffset = {
		{0*DEG_TO_RAD,0*DEG_TO_RAD,0}, -- RPY
		{0.08,0,0.08} -- translation
	},
	w = 512,
	h = 424,
	jpeg_quality = 60,
	-- Use the default vision parameters
	vision = vision_k2,
	-- Run the standard RoboCup vision processing
	detection = 'MultiValve',
	lut = 'multi_valve',
}
if IS_WEBOTS then
	kinect.mountOffset = {
		{0,0,0}, -- RPY
		{0.03,0,0.12} -- translation
	}
end

--Webots use 1/2 resolution but 2x label resolution
if IS_WEBOTS then
	camera[1].w = 320
	camera[1].h = 180
	camera[1].cx_offset = 0
	camera[1].cy_offset = 0
	camera[1].lut = 'webots2'
	--[[
	camera[1].focal_length = 730
	camera[1].focal_base = 640
	--]]

	camera[1].focal_length = 365
	camera[1].focal_base = 320

	-- Kinect width and height
	kinect.w, kinect.h = 256, 212

	vision.scaleA = 2
	vision.scaleB = 2

--[[
	head.neckX = 0 --From CoM to neck joint
	head.cameraPitch = 0
	head.cameraRoll = 0
	head.yawBias = 0
--]]

	vision.goal = {
		th_min_bbox_area = 80,
		th_nPostB = 10,
		th_min_area = 35,
		th_min_orientation = 80*DEG_TO_RAD,
		th_min_fill_rate = 0.28, --0.28,
		height_min = -0.9,  --TODO
		th_aspect_ratio = {2.5,110},
		th_edge_margin = 5,
		th_bottom_boundingbox = 0.9,
		th_ground_boundingbox = {-15,15,-15,10},
		th_min_green_ratio = 0.2,
		th_min_bad_color_ratio = 0.1,
		th_goal_separation = {0.35,3.0},
		th_min_area_unknown_post = 80,
		-- The range we triangulate:
		far_goal_threshold= 4.0,
		use_centerpost = 1,
		min_crossbar_ratio = 0.6,
		check_for_ground = 1,
		height_max = 9,
		-- Metric properties
		goalHeight = 1.8,
		goalWidth = 3.1, --3.0 for I-I, 3.1 for C-C
		postDiameter = 0.1,
	}
end

-- Associate with the table
Config.camera = camera
Config.kinect = kinect
Config.vision = vision
Config.head = head
Config.monitor = monitor

return Config
