#!/usr/bin/env lua
assert(arg, "Run from the shell")
local kind = assert(arg[1], "Specify vision or motion")

dofile'include.lua'
local color = require'util'.color

local status_code
if kind=='motion' then
  print(color("==== STARTING MOTION ===",'magenta'))
  unix.chdir(ROBOT_HOME)
  -- Kill old stuff
  local ps = {}
  local imu_ps = io.popen("pgrep -f run_imu"):lines()
  for p in imu_ps do table.insert(ps, p) end
  local dcm_ps = io.popen("pgrep -f run_co_dcm"):lines()
  for p in dcm_ps do table.insert(ps, p) end
  local state_ps = io.popen("pgrep -f state_wizard"):lines()
  for p in state_ps do table.insert(ps, p) end
  if #ps>0 then
    --print(color("Killing Motion processes...",'red'))
    local kill_cmd = 'kill '..table.concat(ps,' ')
    status_code = os.execute(kill_cmd)
  end
  --
  print("Starting IMU...")
  status_code = os.execute('screen -S imu -L -dm luajit run_imu.lua')
  assert(status_code, 'IMU failed to start')
  unix.usleep(2e6)
  --
  print("Starting DCM...")
  status_code = os.execute('screen -S dcm -L -dm luajit run_co_dcm.lua')
  assert(status_code, 'DCM failed to start')  
  unix.usleep(1e6)
	
  unix.chdir(HOME..'/Player')
	print('state_wizard')
  os.execute('screen -S state -L -dm luajit state_wizard.lua')
  unix.usleep(1e6)

  --
  os.exit()
elseif kind=='vision' then
  print(color("==== STARTING VISION ===",'cyan'))
  unix.chdir(HOME..'/Player')
  -- Kill old stuff
  local ps = {}
  local dcm_ps = io.popen("pgrep -f camera_wizard"):lines()
  for p in dcm_ps do table.insert(ps, p) end
  local dcm_ps = io.popen("pgrep -f world_wizard"):lines()
  for p in dcm_ps do table.insert(ps, p) end
  local dcm_ps = io.popen("pgrep -f gc_wizard"):lines()
  for p in dcm_ps do table.insert(ps, p) end
  if #ps>0 then
    --print(color("Killing Vision processes...",'red'))
    local kill_cmd = 'kill '..table.concat(ps,' ')
    status_code = os.execute(kill_cmd)
  end

  print("Fixing Auto Exposure issue...")
  os.execute('uvcdynctrl -s "Exposure, Auto Priority" 0')
	unix.usleep(1e6)

  print('camera_wizard')
  os.execute('screen -S camera -L -dm luajit camera_wizard.lua')

  print('world_wizard')
  os.execute('screen -S world -L -dm luajit world_wizard.lua')

  print('gamecontrol_wizard')
  os.execute('screen -S gc -L -dm luajit gc_wizard.lua')
end

unix.usleep(1e6)
unix.chdir(HOME..'/Player')
print()
print(color("==== CHECKING STATUS ===",'yellow'))
os.execute'screen -ls'
os.execute'lua check_game.lua'
