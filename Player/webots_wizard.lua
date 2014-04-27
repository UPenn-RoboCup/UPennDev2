---------------------------------
-- State Machine Manager for Team THOR
-- (c) Stephen McGill
---------------------------------
dofile'include.lua'
local Body = require'Body'
require'gcm'

local state_machines = {}
local status = {}

-- TODO: Make coroutines for each FSM
-- TODO: Or other way of handling state machine failure
-- Maybe a reset() function in each fsm?
for _,sm in ipairs(Config.fsm.enabled) do
  local my_fsm = require(sm)
  my_fsm.sm:set_state_debug_handle(function(cur_state_name,event)
    -- For other processes
    gcm['set_fsm_'..sm](cur_state_name)
    -- Local copy
    local s = {cur_state_name,event}
    status[my_fsm._NAME] = s
    -- Broadcast requirement
    --needs_broadcast = true
    -- Debugging printing
    --print(table.concat(s,' from '))
  end)
  state_machines[sm] = my_fsm
  print( 'FSM | Loaded',sm)
end

-- Start the state machines
local t0 = Body.get_time()
local t_debug = t0

-- Entry
Body.entry()
for _,my_fsm in pairs(state_machines) do
	my_fsm.entry()
	local cur_state = my_fsm.sm:get_current_state()
	local cur_state_name = cur_state._NAME
	local s = {cur_state_name,nil}
	status[my_fsm._NAME] = s
end

-- Image Processing
local ImageProc = require'ImageProc'
local ImageProc2 = require'ImageProc.ffi'
local lV = require'libVision'
local lut_id_t = ImageProc2.load_lut (HOME.."/Data/lut_webots.raw")
local lut_top = ImageProc2.get_lut(lut_id_t):data()
local lut_id_b = ImageProc2.load_lut (HOME.."/Data/lut_webots.raw")
local lut_b = ImageProc2.get_lut(lut_id_b):data()
-- TODO: Get this from the config...
local w, h, sA, sB = 320, 240, 2, 2
local nA = (w / sA) * (h / sA)
local nB = (w / sA / sB) * (h / sA / sB)
ImageProc2.setup(w, h)

-- For broadcasting the labeled image
local mp   = require'msgpack.MessagePack'
local zlib = require'zlib.ffi'
local jpeg = require'jpeg'
local c_zlib = zlib.compress_cdata
local c_yuyv = jpeg.compressor'yuyv'
--[[
local udp  = require'udp'
local udp_t = udp.new_sender('127.0.0.1', 33333)
local udp_b = udp.new_sender('127.0.0.1', 33334)
--]]
local simple_ipc = require'simple_ipc'
local s_t = simple_ipc.new_publisher('top')
local s_b = simple_ipc.new_publisher('bottom')

local meta_a = {
  id = 'labelA',
  w = w / sA,
  h = h / sA,
  c = 'zlib',
}
local meta_b = {
  id = 'labelB',
  w = w / sA / sB,
  h = h / sA / sB,
  c = 'zlib',
}
local meta_yuyv = {
  id = 'yuyv',
  w = w,
  h = h,
  c = 'jpeg',
}
local meta_detect = {
  id = 'detect'
}
local meta_world = {
  id = 'world'
}

-- Add Motion from old code
package.path = HOME..'/Player/Motion/?.lua;'..package.path
package.path = HOME..'/Player/Motion/Walk/?.lua;'..package.path
Motion = require'Motion'
Motion.entry()
Motion.event'standup'

-- Process image should essentially be the same code as camera_wizard.lua
-- NOTE: This sleep is important for flushing buffers of udp. Not sure why...
local function process_image(im, lut, id)
  
  -- Data to send on the channel
  local debug_data = {}
  
  -- Images to labels
  local labelA = ImageProc2.yuyv_to_label(im, lut)
  local labelB = ImageProc2.block_bitor(labelA)
  -- Detection System
  -- NOTE: Muse entry each time since on webots, we switch cameras
  -- In camera wizard, we do not switch cameras, so call only once
  local cc = ImageProc2.color_count(labelA)
  lV.entry(Config.vision[id])
  lV.update(Body.get_head_position())
  local ball, ball_v = lV.ball(labelA, labelB, cc)
  -- Send the detection information
  meta_detect.ball = ball
  -- Now add goal detection
  -- Add world coordinates? In arbitrator I guess
  if ball_v then
    --print('ball_v', ball_v)
    meta_world.ball = ball_v
  end

  -- LabelA
  table.insert(debug_data, mp.pack(meta_a)..c_zlib( labelA:data(), nA, true ))
  -- LabelB
  table.insert(debug_data, mp.pack(meta_b)..c_zlib( labelB:data(), nB, true ))
  -- YUYV
  table.insert(debug_data, mp.pack(meta_yuyv)..c_yuyv:compress(im,w,h))
  -- Detection
  table.insert(debug_data, mp.pack(meta_detect))

  return debug_data
end

while true do
  -- Update the body
  Body.update()
  
  -- Image Processing (Must do TOP then BOTTOM fully due to to_rgb pointer)
  local im_top = Body.get_img_top()
  local top_data = process_image(im_top, lut_top, 1)
  local im_b = Body.get_img_bottom()
  local btm_data = process_image(im_b, lut_b, 2)
  
  -- Motion update
  walk.set_velocity(unpack(mcm.get_walk_vel()))
  Motion.update()
  
  -- Update the state machines
	for _,my_fsm in pairs(state_machines) do local event = my_fsm.update() end
  
  -- Send all the debug information
  s_t:send(top_data)
  s_b:send(btm_data)
  
end

-- Exit
for _,my_fsm in pairs(state_machines) do
	my_fsm.exit()
	local cur_state = my_fsm.sm:get_current_state()
	local cur_state_name = cur_state._NAME
	local s = {cur_state_name,nil}
	status[my_fsm._NAME] = s
end

Body.exit()
