-- Vision
-- (c) 2014 Stephen McGill
-- General Detection methods
local Vision = {}
-- Detection and HeadTransform information
local ImageProc = require'ImageProc'
local ImageProc2 = require'ImageProc.ffi2'
local ok, ffi = pcall(require, 'ffi')
local T = require'Transform'
local vector = require'vector'
local util = require'util'
local ptable = require'util'.ptable
require'wcm'
require'hcm'
require'gcm'

local HeadImage
local detectBall
local detectPost
local detectObstacle
local ENABLE_BALL = true
local ENABLE_OBSTACLE = true
local ENABLE_POST = false
local ENABLE_LINE = true

-- Set the variables based on the config file
function Vision.entry(cfg)
	HeadImage = ImageProc2.new(
    cfg.w, cfg.h, cfg.vision.scaleA, cfg.vision.scaleB
  )
	HeadImage:load_lut(table.concat{HOME, "/Data/", "lut_", cfg.lut, ".raw"})
  HeadImage.colors = cfg.vision.colors

  -- Center should be calibrated and saved in Config
  local focal_length, focal_base = cfg.focal_length, cfg.focal_base
  HeadImage.x0A = (HeadImage.wa-1)/2 + cfg.cx_offset
  HeadImage.y0A = (HeadImage.ha-1)/2 + cfg.cy_offset
  HeadImage.x0B = (HeadImage.wb-1)/2 + cfg.cx_offset / HeadImage.scaleB
  HeadImage.y0B = (HeadImage.hb-1)/2 + cfg.cy_offset / HeadImage.scaleB
  HeadImage.focalA = focal_length / (focal_base / HeadImage.wa)
  HeadImage.focalB = HeadImage.focalA / HeadImage.scaleB

  -- Ball
  if cfg.vision.ball and ENABLE_BALL then
		print('Ball detection enabled')
    detectBall = require'detectBall'
    detectBall.entry(cfg.vision.ball, HeadImage)
  end

	-- Obstacles
	if cfg.vision.obstacle and ENABLE_OBSTACLE then
		print('Obstacle detection enabled')
		detectObstacle = require'detectObstacle'
		detectObstacle.entry(cfg.vision.obstacle, HeadImage)
	end

  -- Goal
  if cfg.vision.goal and ENABLE_POST then
		print('Post detection enabled')
    detectPost = require'detectPost'
    detectPost.entry(cfg.vision.goal, HeadImage)
  end

	-- Line
  if cfg.vision.line and ENABLE_LINE then
		print('Line detection enabled')
    detectLine = require'detectLine'
    detectLine.entry(cfg.vision.line, HeadImage)
  end

end

function Vision.update(meta, img)

  -- Images to labels
  --HeadImage:rgb_to_labelA(img)
  HeadImage:yuyv_to_labelA(img)
  HeadImage:block_bitor()
  -- Must always color count
  local cc_d = HeadImage:color_countA()

  -- Grab the metadata
  HeadImage.tfL = T.from_flat(meta.tfL16)
  HeadImage.tfG = T.from_flat(meta.tfG16)
  HeadImage.qHead = vector.new(meta.head)
  HeadImage.t = meta.t

  local ball, b_debug
  if detectBall then
    ball, b_debug = detectBall.update(HeadImage)
  end

  local obs, o_debug
  if detectObstacle then
    obs, o_debug = detectObstacle.update(HeadImage)
	end

	local post, p_debug
  if detectPost then
    post, p_debug = detectPost.update(HeadImage)
  end

	local line, l_debug
  if detectLine then
    line, l_debug = detectLine.update(HeadImage)
  end

  local debug = {
    ball = b_debug or '?',
    post = p_debug or '?',
    obstacle = o_debug or '?',
		line = l_debug or '?',
  }
  local detect = {
    id = 'detect',
    debug = debug
  }
  if ball then detect.ball = ball end
  if post then detect.posts = post end
	if obs then detect.obstacles = obs end
	if line then detect.line = line end

  -- Send the detected stuff over the channel every cycle
  return HeadImage, detect

end

return Vision
