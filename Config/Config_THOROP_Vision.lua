local vector = require'vector'
local DEG_TO_RAD = math.pi/180

local Config = {}

------------------------------------

local camera = {}

------------
-- Cameras --
-------------

-- Head
camera.head = {}
camera.head.device = '/dev/video0'
--camera.head.device = '/dev/video1'
camera.head.format = 'yuyv'
camera.head.resolution = {640,360}
camera.head.x_center = 320
camera.head.y_center = 180
camera.head.focal_length = 533
--hack for THOROP--
camera.head.focal_length = 333
camera.head.focal_base = 640
-- f = 640/2/tan(60/180*pi / 2)
-- OR
-- f = 360/2/tan(45/180*pi / 2) smaller
-- TODO: check C905: diagonal FOV: 75 degree?
camera.head.focal_length = 554.256  --horizontal
-- camera.head.focal_length = 434.56  -- vertical
camera.head.focal_base = 640  -- just use width ratio
-- Logitech C920: horizontal FOV: 78 degree
--------
camera.head.fps = 30




--Head camera parameters

camera.head.auto_param={}
camera.head.auto_param[1] = {key='White Balance Temperature, Auto', val={0}}
camera.head.auto_param[2] = {key='Power Line Frequency',   val={0}}
camera.head.auto_param[3] = {key='Backlight Compensation', val={0}}
camera.head.auto_param[4] = {key='Exposure, Auto',val={1}} --1 for manual
camera.head.auto_param[5] = {key="Exposure, Auto Priority",val={0}}

camera.head.param = {}
-- Wider FOV camera
camera.head.param[1] = {key='Brightness',    val={76}}
camera.head.param[2] = {key='Contrast',      val={117}}
camera.head.param[3] = {key='Saturation',    val={130}}
camera.head.param[4] = {key='Gain',          val={105}}
camera.head.param[5] = {key='White Balance Temperature', val={2885}}
camera.head.param[6] = {key='Sharpness',     val={0}}
camera.head.param[7] = {key='Exposure (Absolute)',      val={90}}
-- Darwin Camera
camera.head.param[1] = {key='Brightness',    val={0}}
camera.head.param[2] = {key='Contrast',      val={10}}
camera.head.param[3] = {key='Saturation',    val={18}}
camera.head.param[4] = {key='Gain',          val={255}}
camera.head.param[5] = {key='White Balance Temperature', val={1000}}
camera.head.param[6] = {key='Sharpness',     val={0}}
camera.head.param[7] = {key='Exposure (Absolute)',      val={2000}}

-- Lookup table
camera.lut_file = 'thorTestGrasp_MonFeb10_14.raw'
camera.lut_file = 'lut_low_contrast_pink_n_green.raw'
camera.lut_file = 'field_C_noon_corrected.raw'  --works best for now
camera.lut_file = '308_wide_night.raw'
camera.lut_file = '308_narrow_night.raw'

camera.jpeg_quality = 60

--[[
-- Forehead (wide angle)
Config.camera.forehead = {}
Config.camera.forehead.device = '/dev/video1'
Config.camera.forehead.format = 'yuyv'
--Config.camera.forehead.format = 'mjpeg'
--Config.camera.forehead.resolution = {160,120}
Config.camera.forehead.resolution = {320,240}
Config.camera.forehead.fps = 10
--]]

--[[
Config.camera.forehead2 = {}
Config.camera.forehead2.device = '/dev/video2'
Config.camera.forehead2.format = 'yuyv'
--Config.camera.forehead2.format = 'mjpeg'
Config.camera.forehead2.resolution = {320,240}
Config.camera.forehead2.fps = 5
--]]



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


vision.ball = {}
vision.ball.diameter = 0.22 
vision.ball.th_min_color = 50 --100
vision.ball.th_min_color2 = 6
vision.ball.th_min_fill_rate = 0.35
vision.ball.th_height_max  = 0.20
vision.ball.th_ground_boundingbox = {-30,30,0,20}
vision.ball.th_min_green1 = 400
vision.ball.th_min_green2 = 150
vision.ball.check_for_ground = 1
vision.ball.check_for_field = 1
vision.ball.field_margin = 2.0


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
