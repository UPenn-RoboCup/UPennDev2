dofile'include.lua'
local Body = require'Body'
local colors = require'colors'

local state_machines = {}
-- TODO: Make coroutines for each FSM
for _,sm in ipairs(unix.readdir(CWD)) do
  if sm:find('LidarFSM') then
    package.path = CWD..'/'..sm..'/?.lua;'..package.path
    state_machines[sm] = require(sm)
  end
end

local entry_str = 'Running'
for name,_ in pairs(state_machines) do entry_str = entry_str..' '..name end
print(colors.wrap(entry_str,'green'))

-- Start the webots routine
local t0 = Body.get_time()
local t_debug = t0
Body.entry()
for _,sm in pairs(state_machines) do sm:entry() end
while true do
  local t = Body.get_time()
  local t_diff = t-t_debug
  
  for _,sm in pairs(state_machines) do sm:update() end
  
  if t_diff>1 then
	  print( string.format('Running Time: %d seconds',t-t0) )
    t_debug = t
  end
  -- NOTE: Body is the main update, so it must not fail
	Body.update()
end

Body.exit()