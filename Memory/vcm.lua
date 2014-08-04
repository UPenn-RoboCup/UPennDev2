---------------------------------
-- Vision Communication Module --
-- (c) 2013 Stephen McGill     --
---------------------------------
local memory = require'memory'
local vector = require'vector'

-- shared properties
local shared = {}
local shsize = {}
-----------------
-- Head Camera --
shared.wire = {}
-- Last time the wire was seen
shared.wire.t = vector.zeros(1)
-- Roll, Pitch Yaw in the camera frame
shared.wire.cam_rpy = vector.zeros(3)
-- Model: Radius and Distance to the wire, from the camera, last model time
shared.wire.model = vector.zeros(3)
-- Bounding box of where we are looking currently
shared.wire.bbox = vector.zeros(4)

-- RoboCup

-- TODO: I really prefer not to have images in shm, or large chunks of memory for that matter
--Added for vision processing (mainly for robocup)
--[[
shared.image={}

-- For head camera
local processed_img_width = Config.camera[1].w
local processed_img_height = Config.camera[1].h

-- local processed_img_width = Config.camera.head.resolution[1]
-- local processed_img_height = Config.camera.head.resolution[2]

shared.image.lut = 262144
shared.image.yuyv = 2*processed_img_width*processed_img_height
shared.image.labelA = (processed_img_width/Config.vision.scaleA)*(processed_img_height/Config.vision.scaleA)
shared.image.labelB = (processed_img_width/Config.vision.scaleB)*(processed_img_height/Config.vision.scaleB)
shsize.image = shared.image.lut + shared.image.yuyv
			+ shared.image.labelA + shared.image.labelB +2^16
--]]
-------------------

shared.ball = {}
shared.ball.centroid = vector.zeros(2)
shared.ball.diameter = vector.zeros(1)
shared.ball.detect = vector.zeros(1)
shared.ball.v = vector.zeros(3)
shared.ball.r = vector.zeros(1)
shared.ball.t = vector.zeros(1)

shared.goal = {}
shared.goal.detect = vector.zeros(1)
shared.goal.enable = vector.zeros(1)
shared.goal.type = vector.zeros(1)
shared.goal.v1 = vector.zeros(4)
shared.goal.v2 = vector.zeros(4)
shared.goal.t = vector.zeros(1)

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
