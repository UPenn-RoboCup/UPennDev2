---------------------------------
-- Vision Communication Module --
-- (c) 2013 Stephen McGill     --
---------------------------------
local memory = require'memory'
local vector = require'vector'
-- TODO: Use the Config file somehow

-- shared properties
local shared = {}
local shsize = {}

local DEG_TO_RAD = math.pi/180

------------------------
--  Head Camera
shared.head_camera       = {}
-- YUYV uses 4 bytes to represent 2 pixels, meaning there are 2 bytes per pixel
--shared.head_camera.image = 2*160*120
-- Look up table is 262144 bytes
--shared.head_camera.lut   = 262144
shared.head_camera.t     = vector.zeros(1)
-- Network Requests: [stream,compression,quality,fps]
-- Stream | 0: None, 1: Single Frame, 2: Stream, 3: Single Reliable Frame
-- Compression | 0: None, 1: JPEG, 2: zlib, 3: PNG
-- Quality | JPEG quality from 0-100
-- Rate | FPS (5,10,15,30, etc.)
shared.head_camera.net = vector.new({2,1,90})

------------------------
--  Head LIDAR
shared.head_lidar           = {}
-- Hokuyo uses a float, which is 4 bytes, to represent each range
shared.head_lidar.scan      = 4*1081
shared.head_lidar.t         = vector.zeros(1)
-- LIDAR parameters: FOV and number of readings
shared.head_lidar.sensor_params = vector.new({270*DEG_TO_RAD, 1081})
-- Radian endpoints for where the lidar is scanning
-- Last one is the resolution: scanlines per radian
shared.head_lidar.scanlines = vector.new({0*DEG_TO_RAD,45*DEG_TO_RAD,10/DEG_TO_RAD})
-- Care only about lidar readings within this field of view
-- {Start angle, stop angle}
shared.head_lidar.fov      = vector.new({-60,60})*DEG_TO_RAD
-- when compressing to 0-255, care about points within these depths
shared.head_lidar.depths      = vector.new({.1,5})
-- Network Requests: [stream,compression,quality,pose]
-- Stream | 0: None, 1: Single Frame, 2: Stream, 3: Single Reliable Frame
-- Compression | 0: None, 1: JPEG, 2: zlib, 3: PNG
-- Quality | JPEG quality from 0-100
-- Pose | Send per-scanline pose information (0: no, 1: yes)
shared.head_lidar.net      = vector.new{0,1,90,1}

------------------------
--  Chest LIDAR
shared.chest_lidar                 = {}
-- Hokuyo uses a float, which is 4 bytes, to represent each range
shared.chest_lidar.scan            = 4*1081
shared.chest_lidar.t               = vector.zeros(1)
-- Radian endpoints for where the lidar is scanning
-- Last one is the resolution: scanlines per radian (we use scanlines per deg conversion)
shared.chest_lidar.scanlines = vector.new({-50*DEG_TO_RAD,50*DEG_TO_RAD,5/DEG_TO_RAD})
-- Care only about lidar readings within this field of view
-- {Start angle, stop angle}
--shared.chest_lidar.fov      = vector.new({-40,80})*DEG_TO_RAD


shared.chest_lidar.fov      = vector.new({-60,80})*DEG_TO_RAD

-- when compressing to 0-255, care about points within these depths
shared.chest_lidar.depths      = vector.new({.1,5})
-- Network Requests: [stream,compression,quality,pose]
-- Stream | 0: None, 1: Single Frame, 2: Stream, 3: Single Reliable Frame
-- Compression | 0: None, 1: JPEG, 2: zlib, 3: PNG
-- Quality | JPEG quality from 0-100
-- Pose | Send per-scanline pose information (0: no, 1: yes)
shared.chest_lidar.net      = vector.new{0,1,90,1}

-- LIDAR parameters: FOV and number of readings
shared.chest_lidar.sensor_params = vector.new({270*DEG_TO_RAD, 1081})

------------------------
--  Slam map
-- Compression format:
-- 0: jpeg   1: zlib
shared.omap = {}
shared.omap.format = vector.zeros(1) 

------------------------
--  Kinect
shared.kinect       = {}
-- Timestamp of last acquisition
shared.kinect.t     = vector.zeros(1)
-- when compressing to 0-255, care about points within these depths
shared.kinect.depths   = vector.new({.5,2})
-- Network Requests: [stream,compression,quality,fps]
-- Stream | 0: None, 1: Single Frame, 2: Stream, 3: Single Reliable Frame
-- Compression | 0: None, 1: JPEG, 2: zlib, 3: PNG
-- Quality | JPEG quality from 0-100
-- Rate | FPS (5,10,15,30, etc.)
shared.kinect.net_color = vector.new{0,1,85}
-- Shift Amount | (Depth only)
shared.kinect.net_depth = vector.new{0,1,95}

-- Customize the shared memory size, due to using userdata
--shsize.head_camera = shared.head_camera.image + shared.head_camera.lut + 2^16
shsize.head_lidar  = shared.head_lidar.scan + 2^16
shsize.chest_lidar = shared.chest_lidar.scan + 2^16

------------------------
-- Ultrasound
shared.us = {}
shared.us.left      = vector.zeros(10)
shared.us.right     = vector.zeros(10)
shared.us.obstacles = vector.zeros(2)
shared.us.free      = vector.zeros(2)
shared.us.dSum      = vector.zeros(2)
shared.us.distance  = vector.zeros(2)

------------------------
-- Initialize the segment
memory.init_shm_segment(..., shared, shsize)
