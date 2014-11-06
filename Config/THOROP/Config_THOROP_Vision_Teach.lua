assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

Config.enable_log = false

local monitor = {
	minFPS = 1,
	maxFPS = 15,
}

-- TODO: put this into FSM config, and state-specific
local head = {
  pitchMin = -10 * math.pi/180,
  pitchMax = 75 * math.pi/180,
  yawMin = -135 * math.pi/180,
  yawMax = 135 * math.pi/180,
	-- Head angle bias
	yawBias = 0,
	-- Camera bias
  cameraPos = {0.045, 0.0, 0.105}, --Post RC14: alvin
  cameraPos = {0.045, 0.0, 0.107}, --Post RC14: TODO: teddy
	cameraPitch = 5*DEG_TO_RAD,
	cameraRoll = 0, --14*DEG_TO_RAD,
  --From CoM to neck joint
  neckZ = .165 + .161,
  neckX = 0,

}

local vision = {
  colors = {
		black = 0,
    orange = 1,
    yellow = 2,
    blue = 4,
    field = 8,
    white = 16,
		cyan = 32,
		magenta = 64,
  },
  scaleA = 2,
  scaleB = 2,
  --TODO: clean up unused stuff
  use_white_wall = 1,
  white_wall_is_blue = 0,
  white_wall_min_count = 3000,
  white_wall_min_rate = 0.3,
  use_nonwhite_wall = 0,
  nonwhite_wall_min_area = 3000,
  nonwhite_wall_max_rate = 0.15,

  --To compensate for the body flexing backward
  --Use this angle instead of walk.bodyTilt
  bodyTilt = 3*DEG_TO_RAD, 

}

vision.goal = {
  th_min_bbox_area = 80, --100, 
  th_nPostB = 15,
  th_min_area = 40,
  th_min_orientation = 80*math.pi/180,
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

-- Cameras
Config.camera = {}

local m308_param = {
	{'White Balance Temperature', 2300},
	{'Exposure (Absolute)', 112},
	{'Focus (absolute)', 0},
	{'Brightness', 128},
	{'Contrast', 128},
	{'Saturation', 220},
	{'Gain', 0},
	{'Sharpness', 0},
}
local grasp_afternoon_param = {
	{'White Balance Temperature', 3300},
	{'Exposure (Absolute)', 170},
	{'Focus (absolute)', 0},
	{'Brightness', 128},
	{'Contrast', 128},
	{'Saturation', 150},
	{'Gain', 66},
	{'Sharpness', 0},
}

table.insert(Config.camera,
  {
    name = 'head',
    dev = '/dev/video0',
    --dev = '/dev/video1',
    format = 'yuyv',
    w = 640,
    h = 360, --480,
    cx_offset = 0,
    cy_offset = 0,
    fps = 30,
    jpeg_quality = 60,
    -- Use the standard head properties
    head = head,
    -- Use the default vision parameters
    vision = vision,
    -- Run the standard RoboCup vision processing
    detection_pipeline = {
      'libVision',
    },
    --Logitech C920
    --lut = 'empty',
    --lut = 'm308_lightson',
		lut = 'ucla1',
		-- f = 640/2/tan(78/180*pi / 2)
		--fov = 2*arctan(d/2f)
		-- f = d/2/tan(fov/2)
		--webots fov: 1.3613

    focal_length = 395.17,
    focal_base = 640,
    auto_param = {
      {'White Balance Temperature, Auto', 0},
      {'Exposure, Auto', 0},
      {'Power Line Frequency', 0},
      {'Exposure, Auto Priority', 0},
			{'Focus, Auto', 0}
    },
		param = m308_param,
    --param = grasp_afternoon_param,
  })

--Webots use 1/2 resolution but 2x label resolution
if IS_WEBOTS then
  Config.camera[1].w = 320
  Config.camera[1].h = 180
  Config.camera[1].lut = 'wbt_chopsticks'

  Config.camera[1].cx_offset = 0
  Config.camera[1].cy_offset = 0

  focal_length = 395.17
  focal_base = 640
  
	head.neckX = 0 --From CoM to neck joint

  vision.scaleA = 2
  vision.scaleB = 2
  
  head.cameraPitch = 0
  head.cameraRoll = 0
	head.yawBias = 0
  
  vision.goal = {
    th_min_bbox_area = 80, 
    th_nPostB = 10,
    th_min_area = 35,
    th_min_orientation = 80*math.pi/180,
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
Config.vision = vision
Config.head = head
Config.monitor = monitor

return Config
