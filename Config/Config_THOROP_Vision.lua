assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

-------------
-- Cameras --
-------------

Config.camera = {
  jpeg_quality = 60,
}
table.insert(Config.camera,
  {
    name = 'head',
    dev = '/dev/video0',
    format = 'yuyv',
    w = 640,
    h = 360, --480,
    fps = 30,
    udp_port = 33333,
    lA_port = 33335,
    --[[LogiTech C920
    lut = '308_wide_night.raw',
    focal_length = 533,
    focal_base = 640,
    auto_param = {
      {'White Balance Temperature, Auto', 0},
      {'Power Line Frequency', 0},
      {'Exposure, Auto', 1},  -- 1 for manual?
      {'Exposure, Auto Priority', 0},
    },
    param = {
      {'Brightness', 76},
      {'Contrast', 117},
      {'Saturation', 130},
      --{'Exposure (Absolute)', 90},
      {'Gain', 105},
      {'Sharpness', 0},
      {'White Balance Temperature', 2885},
      --{'Backlight Compensation', 1},
    },
    --]]
    ---[[LogiTech C905
    lut = '308_narrow_night.raw',
    focal_length = 554.256,
    focal_base = 640,
    auto_param = {
      {'White Balance Temperature, Auto', 0},
      {'Power Line Frequency', 0},
      {'Exposure, Auto', 1},
      {'Exposure, Auto Priority', 0},
    },
    param = {
      {'Brightness', 0},
      {'Contrast', 10},
      {'Saturation', 18},
      {'Exposure (Absolute)', 800},
      {'Gain', 255},
      {'Sharpness', 0},
      {'White Balance Temperature', 1000},
    },
    --]]
  })

local vision = {}

vision.scaleA = 2
vision.scaleB = 4

vision.colorOrange = 1
vision.colorYellow = 2
vision.colorCyan = 4
vision.colorField = 8
vision.colorWhite = 16
--TODO: vision.colorBlack for obstacle

vision.use_white_wall = 1
vision.white_wall_is_blue = 0 --white wall is on the red side
vision.white_wall_min_count = 3000  --TODO
vision.white_wall_min_rate = 0.3

vision.use_nonwhite_wall = 0 --disabled
vision.nonwhite_wall_min_area = 3000
vision.nonwhite_wall_max_rate = 0.15


vision.ball = {
  diameter = 0.22,
  th_min_color = 50, --100
  th_min_color2 = 6,
  th_min_fill_rate = 0.35,
  th_height_max  = 0.20,
  th_ground_boundingbox = {-30,30,0,20},
  th_min_green1 = 400,
  th_min_green2 = 150,
  check_for_ground = 1,
  check_for_field = 1,
  field_margin = 2.0,
}

vision.goal={}
vision.goal.th_min_color = 120 --TODO
vision.goal.th_nPostB = 5
vision.goal.th_min_area = 40
vision.goal.th_min_orientation = 60*math.pi/180
vision.goal.th_min_fill_rate = 0.4 --TODO
vision.goal.height_min = -0.9 --bodyHeight: 0.928
vision.goal.th_aspect_ratio = {2.5,35}
vision.goal.th_edge_margin = 5
vision.goal.th_bottom_boundingbox = 0.9
vision.goal.th_ground_boundingbox = {-15,15,-15,10}
vision.goal.th_min_green_ratio = 0.2
vision.goal.th_min_bad_color_ratio = 0.1
vision.goal.th_goal_separation = {0.35,2.0}
vision.goal.th_goal_separation = {0.35,3.0} --FOR OP
vision.goal.th_min_area_unknown_post = 100  -- TODO

vision.goal.far_goal_threshold= 4.0 --The range we triangulate
--vision.goal.distanceFactorCyan = 1
--vision.goal.distanceFactorYellow = 1
vision.goal.use_centerpost = 1
vision.goal.min_crossbar_ratio = 0.6 -- TODO
vision.goal.check_for_ground = 1


-- Black obstacle
vision.obstacle = {}
vision.obstacle.th_min_color = 300
vision.obstacle.th_min_area = 60  --TODO
vision.obstacle.th_min_orientation = 60*math.pi/180
vision.obstacle.th_fill_rate = 0.5 -- TODO

vision.obstacle.height_min = -0.9
vision.obstacle.height_max = -0.5  --TODO: distracting objects..
vision.obstacle.th_aspect_ratio = {2.5, 6}


-- Line
line = {}
line.min_white_pixel = 300
line.min_green_pixel = 5000
line.max_width = 8
line.connect_th = 1.4
line.max_gap = 1
line.min_count = 20  -- labelB
line.min_length = 10  -- labelB
line.max_height = 0.3 -- meters
line.min_aspect_ratio = 2.5
line.min_angle_diff = 15
line.max_angle_diff = 85
vision.line = line


local head = {}
head.pitchMin = 0*math.pi/180
head.pitchMax = 45*math.pi/180
head.yawMin = -135*math.pi/180
head.yawMax = 135*math.pi/180
head.cameraPos = {0.034, 0.0, 0.1} --TODO:need to be recalibrated
head.cameraAngle = {0.0,0} -- We set it zero here
head.neckZ=.165+.161 --From CoM to neck joint; for darwin camera
head.neckZ=.165+.161 --TODO: for camera of wider FOV
head.neckX= 0 --From CoM to neck joint
------------------------------------


WEBOTS_VISION = true
WEBOTS_VISION = false

--Webots use 1/2 resolution but 2x label resolution
if WEBOTS_VISION then
  camera.head.resolution = {320,180}
  camera.head.x_center = 160
  camera.head.y_center = 90
  camera.head.focal_length = 554.256/2

  camera.head.focal_base = 320
	head.neckX= 0 --From CoM to neck joint

  head.cameraPos = {0.0785, 0, 0.072}

  vision.scaleA = 1
  vision.scaleB = 4
  -- Karen: smaller labelA resolution reduces vcm burden
  vision.scaleA = 2
  vision.scaleB = 2
  camera.lut_file = 'lutWebots74.raw'
end


-- Associate with the table
Config.camera = camera
Config.vision = vision
Config.head = head

return Config
