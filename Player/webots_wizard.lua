--------------------------------
--       Webots Manager       --
-- (c) Stephen McGill, Qin He --
--------------------------------
dofile'include.lua'
--require'gcm'
local Body = require(Config.dev.body)
local lW = require'libWorld'
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
-- Subscribe to important messages
local vision_ch = si.new_subscriber'vision'
local util = require'util'
-- SHM
require'wcm'
require'mcm'
require'hcm'

-- UDP channel
local udp = require'udp'
local operator = Config.net.operator.wired
local udp_ch = udp.new_sender(operator, Config.camera[1].udp_port)

-- Cache some functions
local get_time, usleep = Body.get_time, unix.usleep

-- Cleanly exit on Ctrl-C
local running, signal = true, nil

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

local vector = require'vector'

local uOdometry0, uOdometry
local t_send, SEND_INTERVAL = 0




vision_ch.callback = function(skt)
  local detections = skt:recv_all()
  -- First, update the odometry

--Should use the differential of odometry!
  if not uOdometry0 then uOdometry0 = mcm.get_status_odometry()
  else uOdometry0 = uOdometry end
  uOdometry = mcm.get_status_odometry()
  dOdometry = util.pose_relative(uOdometry,uOdometry0)

  -- Only use the last vision detection
	local detection = mp.unpack(detections[#detections])

  lW.update(dOdometry, detection)

  -- Send localization info to monitor
  local t = get_time()
  if t-t_send > SEND_INTERVAL then

--  print("World data sent at fps:",1/(t-t_send))
    local metadata = {}
    metadata.id = 'world'
    metadata.world = lW.send()
    -- Send!
    udp_ch:send(mp.pack(metadata)) 
    t_send = t
  end

end


if not Config.fsm.disabled then load_fsm() end

-- Timing
local t_sleep = 1 / Config.fsm.update_rate
local t0, t = get_time()
local debug_interval, t_debug = 2.0, t0

-- Entry
for _, my_fsm in pairs(state_machines) do
  my_fsm:entry()
end


-- Timing
local TIMEOUT = 1 / 10
-- Timeout in milliseconds
local TIMEOUT_MS = TIMEOUT * 1e3
local poller = si.wait_on_channels{vision_ch}
local npoll

-- Entry
Body.entry()
lW.entry()

gcm.set_game_role(2) --Testing
gcm.set_game_state(5) --Pre-init


--[[
gcm.set_game_role(1) --Attacker
gcm.set_game_state(5) --Pre-init
--]]




-- Update loop
while running do
  SEND_INTERVAL = 1 / hcm.get_monitor_fps()
  npoll = poller:poll(TIMEOUT_MS)
  if npoll==0 then
    -- If no frames, then just update by odometry
    --Should use the differential of odometry!
    if not uOdometry0 then uOdometry0 = mcm.get_status_odometry()
    else uOdometry0 = uOdometry end
    uOdometry = mcm.get_status_odometry()
    dOdometry = util.pose_relative(uOdometry,uOdometry0)
    lW.update_odometry(dOdometry)

    -- Update the pose here
    wcm.set_robot_pose(lW.get_pose())
  end
  
  t = get_time()
  -- Update the state machines
  for _,my_fsm in pairs(state_machines) do my_fsm:update() end
  -- If time for debug
  if t-t_debug>debug_interval then
    t_debug = t
    if Config.debug.webots_wizard then
		  print(string.format('State | Uptime: %.2f sec, Mem: %d kB', t-t0, collectgarbage('count')))
    end
	end
  
  -- If not webots, then wait the update cycle rate
  Body.update()
end

-- Exit
print'Exiting state wizard...'
for _,my_fsm in pairs(state_machines) do
  my_fsm:exit()
end

lW.exit()
os.exit()
