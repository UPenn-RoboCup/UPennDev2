assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

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
    orange = 1,
    yellow = 2,
    cyan = 4,
    field = 8,
    white = 16,
    black = 0,
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

  --
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
}

-- Testing in M308 and grasp with thinner obstacles
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

-- Line
vision.line = {
  -- min_white_pixel = 300,
  -- min_green_pixel = 5000,
  max_width = 15,
  connect_th = 1.4,
  max_gap = 1,
  -- labelB space
  min_count = 20,
  min_length = 5,--10,
  max_height = 0.3,
  min_aspect_ratio = 2.5,
  min_angle_diff = 10,
  max_angle_diff = 85,
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
		lut = 'ucla0',
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
  Config.camera[1].lut = 'webots'

  Config.camera[1].cx_offset = 0
  Config.camera[1].cy_offset = 0

  focal_length = 395.17
  focal_base = 640
  
	head.neckX= 0 --From CoM to neck joint

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
  }
  
  vision.obstacle = {
    label = 'b',
    grid_x = 10, 
    grid_y = 15,
    th_min_area = 42,
    th_min_orientation = 70/180*math.pi,
    th_aspect_ratio = {2, 8},
    min_black_fill_rate = 0.4,

    th_max_height = 1.25,
    th_min_height = -0.2,
    min_ground_fill_rate = 0.55,
		--
		min_width = 3, 
		max_width = 15,
  }
  
  vision.ball = {
  diameter = 0.22,
  th_min_bbox_area = 10,
  th_min_area = 5,
  th_min_fill_rate = 0.35,
  max_height0 = 0.34,    --Max height = max_height0 + dist*max_height1
  max_height1 = 0.19,
  max_distance = 9, 
  th_ground_head_pitch = 50*DEG_TO_RAD,
  th_ground_boundingbox = {-30,30,0,20},
  th_ground_green = 400,  --TODO
  th_ground_white = 150,  --TODO
  check_for_ground = 1,
  check_for_field = 1,
  field_margin = 2.0,
}
end

-- Associate with the table
Config.vision = vision
Config.head = head
Config.monitor = monitor

return Config
