---------------------------
-- State Machine Manager --
-- (c) Stephen McGill    --
---------------------------
dofile'include.lua'
--require'gcm'
local Body = require(Config.dev.body)
-- Cache some functions
local get_time, usleep = Body.get_time, unix.usleep

-- Cleanly exit on Ctrl-C
local running, signal = true, nil
if not IS_WEBOTS then
  signal = require'signal'
  function shutdown ()
    running = false
    --os.exit()
  end
  signal.signal("SIGINT", shutdown)
  signal.signal("SIGTERM", shutdown)
end

-- Load the FSMs and attach event handler
local state_machines = {}
local function load_fsm ()
  for _,sm in ipairs(Config.fsm.enabled) do
    local my_fsm = require(sm..'FSM')
    local set_gcm_fsm = gcm and gcm['set_fsm_'..sm]
    if set_gcm_fsm then
      my_fsm.sm:set_state_debug_handle(function(cur_state_name, event)
        set_gcm_fsm(cur_state_name)
      end)
      set_gcm_fsm('UNKNOWN')
    end
    state_machines[sm] = my_fsm
    print('State | Loaded', sm)
  end
end

if not Config.fsm.disabled then load_fsm() end

-- Timing
local t_sleep = 1 / Config.fsm.update_rate
local t0, t = get_time()
local debug_interval, t_debug = 2.0, t0

-- Channels for monitoring
local udp = require'udp'
local udp_ch = udp.new_sender(Config.net.operator.wired, Config.camera[1].udp_port)
local lA_ch = udp.new_sender(Config.net.operator.wired, Config.camera[1].lA_port)
-- local detect_ch = udp.new_sender(operator, Config.net.detect)


require'vcm'
local ImageProc = require'ImageProc'
if jit then ImageProc2 = require'ImageProc.ffi' end
local HT = require'HeadTransform'
local detectBall = require'detectBall'
local detectGoal = require'detectGoal'
local World = require'World'
local jpeg = require'jpeg'
local c_yuyv = jpeg.compressor('yuyv')

-- Define Color
local colorOrange = Config.vision.colorOrange
local colorYellow = Config.vision.colorYellow
local colorWhite = Config.vision.colorWhite


local labelA, labelB, scaleA, scaleB, lut
local ball, goal, line = {}, {}, {}

local function initialize()
  labelA,labelB = {},{}
  scaleA, scaleB = Config.vision.scaleA, Config.vision.scaleB
	print('NEW LABELING!!')

  ImageProc2.setup(w, h, scaleA, scaleB)
  -- Load colortable
  local lut_filename = HOME.."Data/lut_webots.raw"
  local _, lut_id = ImageProc2.load_lut(lut_filename)
  lut = ImageProc2.get_lut(lut_id):data() -- TODO
  print('LOADED '..lut_filename)

	ball.detect, goal.detect, line.detect = 0,0,0
	vcm.set_ball_detect(0)
	vcm.set_goal_detect(0)
	vcm.set_goal_type(0)
	vcm.set_goal_enable(0)
	
	--World.entry()
end

local function process_image(img, lut)
	-- Generate label
	labelA_t = ImageProc2.yuyv_to_label(img, lut)
	labelB_t = ImageProc2.block_bitor(labelA_t)
	cc_t = ImageProc2.color_count(labelA_t)
	-- convert to light userdata
	--TODO: get rid of this, just use torch tensor
	local cutil = require'cutil'
	labelA.data = cutil.torch_to_userdata(labelA_t)
	colorCount  = cc_t
	labelB.data = cutil.torch_to_userdata(labelB_t)

	-- Label param
	labelA.m, labelA.n = w/scaleA, h/scaleA
	labelA.npixel = labelA.m * labelA.n
	labelB.m, labelB.n = labelA.m/scaleB, labelA.n/scaleB
	labelB.npixel = labelB.m * labelB.n
  
end

-- Entry
for _, my_fsm in pairs(state_machines) do
  my_fsm:entry()
end

Body.entry()
Body.update()
initialize()

-- Update loop
while running do
  t = get_time()
  -- Update the state machines
  for _,my_fsm in pairs(state_machines) do my_fsm:update() end
  -- If time for debug
  if t-t_debug>debug_interval then
    t_debug = t
		print(string.format('State | Uptime: %.2f sec, Mem: %d kB', t-t0, collectgarbage('count')))
    --print('Wire', vcm.get_wire_model())
	end

  Body.update()
  
  -- Image Processing (Must do TOP then BOTTOM fully due to to_rgb pointer)
  local im_yuyv = Body.get_image()
  local img_debug, img_detection
  if im_yuyv then
    img_debug, img_detection = process_image(im_yuyv, lut_top)
  end
    
    
  -- local headAngles = Body.get_head_position()
  -- HT.update(headAngles, labelA, labelB, metadata.focal_length, metadata.focal_base)
  -- 
  -- -- Detection
  -- ball = detectBall.detect(colorOrange,colorCount,labelA,labelB,HT,t)
  -- goal = detectGoal.detect(colorYellow,colorCount,labelA,labelB,HT,t)
	
	
	-- Check if we are sending stuff to operator
	if udp_ch then
		local c_img = c_yuyv:compress(img, w, h)
		meta.sz = #c_img
		local udp_ret, err = udp_ch:send( mp.pack(meta)..c_img )
		if err then print(err, udp_ret) end
	end

	if lA_ch then
		local wA, hA = labelA.m, labelA.n
		local labelA_img = ImageProc.label_to_yuyv(labelA.data,wA,hA)
		local c_img = c_yuyv:compress(labelA_img, wA, hA)
		meta.width, meta.height, meta.sz = wA, hA, #c_img
		local udp_ret, err = lA_ch:send(mp.pack(meta)..c_img)
		if err then print('label udp err', err) end
	end

	if detect_ch then
		send_overlay(ball, goal, line) -- TODO:world
	end
  
    
end

-- Exit
print'Exiting state wizard...'
for _,my_fsm in pairs(state_machines) do
  my_fsm:exit()
end

os.exit()
