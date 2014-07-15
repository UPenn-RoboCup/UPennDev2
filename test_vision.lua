dofile'include.lua'

-- Important libraries in the global space
local libs = {
  'Config',
  'Body',
  'unix',
  'util',
  'vector',
	'fun',
}

-- Load the libraries
for _,lib in ipairs(libs) do _G[lib] = require(lib) end
if torch then torch.Tensor = torch.DoubleTensor end
local getch=require'getch'
-- ffi
local ok, ffi = pcall(require,'ffi')
local si = require'simple_ipc'

-- FSM communicationg
local fsm_chs = {}

for _,sm in ipairs(Config.fsm.enabled) do
  local fsm_name = sm..'FSM'
  table.insert(fsm_chs, fsm_name)
  _G[sm:lower()..'_ch'] = si.new_publisher(fsm_name.."!")
end

require'gcm'
require'wcm'
require'hcm'

-- Head angle limits
local pitchMin = Config.head.pitchMin
local pitchMax = Config.head.pitchMax
local yawMin = Config.head.yawMin
local yawMax = Config.head.yawMax

local head_now = Body.get_head_position()
local head_new = head_now

-- Camera pitch
local cameraPitch = hcm.get_camera_pitch()
local cameraRoll = hcm.get_camera_roll()
local camera_changed = false
-- Head angle bias
local headYawBias = hcm.get_headbias_yaw()
local head_changed = false
-- Monitor FPS
local monitor_fps = hcm.get_monitor_fps()
local monitor_fps_new = monitor_fps
local minFPS, maxFPS = Config.monitor.minFPS, Config.monitor.maxFPS
local dFPS = 3

local dq = 5*DEG_TO_RAD
function process_keyinput()
  local byte=getch.block();
  if byte then
    -- Head angle setting
    if byte==string.byte("w") then head_new[2]=head_now[2]+ dq
    elseif byte==string.byte("x") then head_new[2]=head_now[2] - dq
    elseif byte==string.byte("d") then head_new[1]=head_now[1] - dq
    elseif byte==string.byte("a") then head_new[1]=head_now[1] + dq
    elseif byte==string.byte("s") then head_new = {0,0}
    
	  -- Head FSM
    elseif byte==string.byte("1") then
      body_ch:send'init'      
    elseif byte==string.byte("2") then
      head_ch:send'teleop'
    elseif byte==string.byte("3") then    
      head_ch:send'scan'
		elseif byte==string.byte("4") then    
			wcm.set_obstacle_enable(1)
      head_ch:send'scanobs'
		elseif byte==string.byte("o") then    
			wcm.set_obstacle_enable(1)
			wcm.set_obstacle_reset(1)
		
		-- camera angles
		elseif byte==string.byte("i") then
			camera_changed = true
			cameraPitch = hcm.get_camera_pitch() + 1*DEG_TO_RAD
			hcm.set_camera_pitch(cameraPitch)
		elseif byte==string.byte(",") then
			camera_changed = true
			cameraPitch = hcm.get_camera_pitch() - 1*DEG_TO_RAD
			hcm.set_camera_pitch(cameraPitch)
		elseif byte==string.byte("j") then
			camera_changed = true
			cameraRoll = hcm.get_camera_roll() + 1*DEG_TO_RAD
			hcm.set_camera_roll(cameraRoll)
		elseif byte==string.byte("l") then
			camera_changed = true
			cameraRoll = hcm.get_camera_roll() - 1*DEG_TO_RAD
			hcm.set_camera_roll(cameraRoll)
		elseif byte==string.byte("k") then
			camera_changed = true
			cameraPitch, cameraRoll = 0, 0
			hcm.set_camera_pitch(cameraPitch)
			hcm.set_camera_roll(cameraRoll)

		-- Head angles offset
		elseif byte==string.byte("[") then
			head_changed = true
			headYawBias = hcm.get_headbias_yaw() + 0.05*DEG_TO_RAD
			hcm.set_headbias_yaw(headYawBias)
		elseif byte==string.byte("]") then
			head_changed = true
			headYawBias = hcm.get_headbias_yaw() - 0.05*DEG_TO_RAD
			hcm.set_headbias_yaw(headYawBias)

		-- Frame rate for monitoring
    elseif byte==string.byte("=") then
			monitor_fps_new = monitor_fps + dFPS
		elseif byte==string.byte("-") then
			monitor_fps_new = monitor_fps - dFPS
    end
    
    -- Clamp
    head_new[1] = math.min(math.max(head_new[1], yawMin), yawMax)
    head_new[2] = math.min(math.max(head_new[2], pitchMin), pitchMax)
		monitor_fps_new = math.min(math.max(monitor_fps_new, minFPS), maxFPS)
    -- Update hcm
    hcm.set_motion_headangle(head_new)
		hcm.set_monitor_fps(monitor_fps_new)
    head_now = head_new
		monitor_fps = monitor_fps_new
    
    print(string.format("Target head angle:   %.1f   %.1f  degree \n",
      head_new[1]*RAD_TO_DEG, head_new[2]*RAD_TO_DEG))
    
    print(string.format("Monitor FPS:   %.1f", monitor_fps_new))

		if camera_changed then
			camera_changed = false
			print(string.format("Camera pitch: %.1f  deg,  roll: %.1f  deg \n",
      	cameraPitch*RAD_TO_DEG, cameraRoll*RAD_TO_DEG))
		end
		if head_changed then
			head_changed = false
			print(string.format("Head yaw bias: %.2f  deg\n",
      	headYawBias*RAD_TO_DEG))
		end

  end
end

local t_last = Body.get_time()
local tDelay = 0.005*1E6

while true do
  process_keyinput(); --why nonblocking reading does not work?
end
