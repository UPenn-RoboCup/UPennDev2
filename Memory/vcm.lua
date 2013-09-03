---------------------------------------------------------------------------
-- Vision Communication Module
-- (c) 2013 Stephen McGill
---------------------------------------------------------------------------
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
shared.head_camera.image = 2*160*120
-- Look up table is 262144 bytes
shared.head_camera.lut   = 262144
shared.head_camera.t     = vector.zeros(1)
shared.head_camera.net = vector.zeros(3)

------------------------
--  Head LIDAR
shared.head_lidar           = {}
-- Hokuyo uses a float, which is 4 bytes, to represent each range
shared.head_lidar.scan      = 4*1081
shared.head_lidar.t         = vector.zeros(1)
-- Radian endpoints for where the lidar is scanning
-- Last one is the resolution: scanlines per radian
shared.head_lidar.scanlines = vector.new({0*DEG_TO_RAD,45*DEG_TO_RAD,10/DEG_TO_RAD})
-- Care only about lidar readings within this field of view
-- {Start angle, stop angle}
shared.head_lidar.fov      = vector.new({-45,45})*DEG_TO_RAD
-- when compressing to 0-255, care about points within these depths
shared.head_lidar.depths      = vector.new({.1,5})
-- Network Requests: [stream,compression,fps]
-- Stream | 0: None, 1: Single Frame, 2: Stream
-- Compression | 0: None, 1: JPEG, 2: zlib
-- Interval | Frames per second
shared.head_lidar.net      = vector.zeros(3)

------------------------
--  Chest LIDAR
shared.chest_lidar                 = {}
-- Hokuyo uses a float, which is 4 bytes, to represent each range
shared.chest_lidar.scan            = 4*1081
shared.chest_lidar.t               = vector.zeros(1)
-- Radian endpoints for where the lidar is scanning
-- Last one is the resolution: scanlines per radian
shared.chest_lidar.scanlines = vector.new({-50*DEG_TO_RAD,50*DEG_TO_RAD,10/DEG_TO_RAD})
-- Care only about lidar readings within this field of view
-- {Start angle, stop angle}
shared.chest_lidar.fov      = vector.new({-60,60})*DEG_TO_RAD
-- when compressing to 0-255, care about points within these depths
shared.chest_lidar.depths      = vector.new({.1,5})
-- Network Requests: [stream,compression,fps]
-- Stream | 0: None, 1: Single Frame, 2: Stream
-- Compression | 0: None, 1: JPEG, 2: zlib
-- Interval | Frames per second
shared.chest_lidar.net      = vector.zeros(3)

------------------------
--  Kinect
shared.kinect       = {}
-- Timestamp of last acquisition
shared.kinect.t     = vector.zeros(1)
-- RGB for the kinect
-- Look up table is 2^18 bytes
shared.kinect.lut   = 262144
-- Network Requests: [stream,compression,fps]
-- Stream | 0: None, 1: Single Frame, 2: Stream
-- Compression | 0: None, 1: JPEG, 2: PNG
-- Interval | Frames per second
shared.kinect.net_color = vector.zeros(3)
shared.kinect.net_depth = vector.zeros(3)

-- Customize the shared memory size, due to using userdata
shsize.head_camera = shared.head_camera.image + shared.head_camera.lut + 2^16
shsize.head_lidar  = shared.head_lidar.scan + 2^16
shsize.chest_lidar = shared.chest_lidar.scan + 2^16
shsize.kinect      = shared.kinect.lut + 2^16
--shsize.kinect      = shared.kinect.color + shared.kinect.depth + shared.kinect.lut + 2^16

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