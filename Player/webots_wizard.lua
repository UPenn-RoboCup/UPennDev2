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
local w, h, sA = 320, 240, 2
local nA = (w / sA) * (h / sA)
ImageProc2.setup(w, h)

-- For broadcasting the labeled image
local mp   = require'msgpack.MessagePack'
local zlib = require'zlib.ffi'
local jpeg = require'jpeg'
local udp  = require'udp'
local c_zlib = zlib.compress_cdata
local c_yuyv = jpeg.compressor'yuyv'
local udp_t = udp.new_sender('127.0.0.1', 33333)
local udp_b = udp.new_sender('127.0.0.1', 33334)

local meta_a = {
  w = w / sA,
  h = h / sA,
  c = 'zlib',
}
local meta_j = {
  w = w,
  h = h,
  c = 'jpeg',
}

-- Process image should essentially be the same code as camera_wizard.lua
-- NOTE: This sleep is important for flushing buffers of udp. Not sure why...
local function process_image(im, lut, udp, id)
  -- Images to labels
  local labelA = ImageProc2.yuyv_to_label(im, lut)
  local labelB = ImageProc2.block_bitor(labelA)
  -- Send images and labels to monitor
  udp:send( mp.pack(meta_a)..c_zlib( labelA:data(), nA, true ) )
  unix.usleep(1e4)
  udp:send( mp.pack(meta_j)..c_yuyv:compress(im,w,h) )
  unix.usleep(1e4)
  -- Detection System
  -- NOTE: Muse entry each time since on webots, we switch cameras
  -- In camera wizard, we do not switch cameras, so call only once
  local cc = ImageProc2.color_count(labelA)
  lV.entry(Config.vision[id])
  lV.update(Body.get_head_position())
  local ball = lV.ball(labelA, labelB, cc)
  -- Send the detection information
  local meta_detect = {}
  meta_detect.ball = ball
  udp:send( mp.pack(meta_detect) )
  unix.usleep(1e4)
end

while true do
  -- Update the body
  Body.update()
  -- Image Processing (Must do TOP then BOTTOM fully due to to_rgb pointer)
  local im_top = Body.get_img_top()
  process_image(im_top, lut_top, udp_t, 1)
  local im_b = Body.get_img_bottom()
  process_image(im_b, lut_b, udp_b, 2)
  
  -- Update the state machines
	for _,my_fsm in pairs(state_machines) do local event = my_fsm.update() end
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
