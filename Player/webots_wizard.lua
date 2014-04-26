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
local lut_id = ImageProc2.load_lut (HOME.."/Data/lut_webots.raw")
local lut = ImageProc2.get_lut(lut_id):data()
local w, h, sA = 320, 240, 2
ImageProc2.setup(w, h)

local a_sz = (w / sA) * (240 / sA)

-- For broadcasting the labeled image
local zlib = require'zlib.ffi'
local c_zlib = zlib.compress_cdata
local mp    = require'msgpack.MessagePack'
local udp = require'udp'
local udp_ch = udp.new_sender('127.0.0.1', 33333)
local jpeg = require'jpeg'
local c_yuyv = jpeg.compressor'yuyv'

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

while true do
  -- Update the body
  Body.update()
  -- Image Processing
  local im_top, im_btm = Body.get_images()
  local labelA_t = ImageProc2.yuyv_to_label(im_top, lut)
  local cc_t = ImageProc2.color_count(labelA_t)
  local labelB_t = ImageProc2.block_bitor(labelA_t)
  -- Send images to monitor
  lA_z = c_zlib( labelA_t:data(), a_sz, true )
  local udp_ret, err = udp_ch:send( mp.pack(meta_a)..lA_z )
  -- Send JPEG image
  --yuyv_j = c_yuyv:compress(yuyv_t,w,h)
  --local udp_ret, err = udp_ch:send( mp.pack(meta_j)..yuyv_j )
  
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
	--local ret = state_pub_ch:send( mp.pack(status) )
end

Body.exit()
