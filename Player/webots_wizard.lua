dofile'include.lua'
local Body = require'Body'
local util = require'util'

local state_machines = {}
-- TODO: Make coroutines for each FSM
for _,sm in ipairs(unix.readdir(CWD)) do
  if sm:find('LidarFSM') or sm:find('MotionFSM') or sm=='ArmFSM' then
    package.path = CWD..'/'..sm..'/?.lua;'..package.path
    state_machines[sm] = require(sm)
    print('Using FSM',sm)
  end
end

local entry_str = 'Running'
for name,_ in pairs(state_machines) do entry_str = entry_str..' '..name end
print(util.color(entry_str,'green'))

-- Start the webots routine
local t0 = Body.get_time()
local t_debug = t0

-- Update rate
local fps = 100
local us_sleep = 1e6 / fps

Body.entry()
for _,sm in pairs(state_machines) do sm.entry() end
while true do
  local t = Body.get_time()
  local t_diff = t-t_debug
  
  -- Update each state machine
  for _,sm in pairs(state_machines) do sm.update() end

  if t_diff>1 then
	  print( string.format('Running Time: %d seconds',t-t0) )
    t_debug = t
  end
  -- NOTE: Body is the main update, so it must not fail
	Body.update()
  
  -- Sleep a bit if not webots
  if not IS_WEBOTS then unix.usleep(us_sleep) end
  
end

Body.exit()