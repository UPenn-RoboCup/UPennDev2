---------------------------
-- State Machine Manager --
-- (c) Stephen McGill    --
---------------------------
dofile'include.lua'
--require'gcm'
local Body = require(Config.dev.body)
local lW = require'libWorld'
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
-- Subscribe to important messages
local vision_ch = si.new_subscriber'vision'
-- SHM
require'wcm'
require'mcm'

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

local uOdometry
vision_ch.callback = function(skt)
  local detections = skt:recv_all()
  -- First, update the odometry
  uOdometry = mcm.get_status_odometry()
  -- Only use the last vision detection
	local detection = mp.unpack(detections[#detections])

  lW.update(uOdometry, detection)
  
	local pose = lW.get_pose()
  wcm.set_robot_pose(pose)
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

-- Update loop
while running do
  npoll = poller:poll(TIMEOUT_MS)
  if npoll==0 then
    -- If no frames, then just update by odometry
    uOdometry = mcm.get_status_odometry()
    lW.update_odometry(uOdometry)
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
    --print('Wire', vcm.get_wire_model())
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
