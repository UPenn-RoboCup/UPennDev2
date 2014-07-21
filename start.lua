#!/usr/bin/env lua
assert(arg, "Run from the shell")
local kind = assert(arg[1], "Specify vision or motion")

dofile'include.lua'

local status_code
if kind=='motion' then
  unix.chdir(ROBOT_HOME)
  -- Kill old stuff
  local ps = {}
  local imu_ps = io.popen("pgrep -f run_imu"):lines()
  for p in imu_ps do table.insert(ps, p) end
  local dcm_ps = io.popen("pgrep -f run_co_dcm"):lines()
  for p in dcm_ps do table.insert(ps, p) end
  if #ps>0 then
    print("Killing Motion processes...")
    local kill_cmd = 'kill '..table.concat(ps,' ')
    status_code = os.execute(kill_cmd)
  end
  --
  print("Starting IMU...")
  status_code = os.execute('screen -S imu -L -dm luajit run_imu.lua')
  assert(status_code, 'IMU failed to start')
  --
  print("Starting DCM...")
  status_code = os.execute('screen -S dcm -L -dm luajit run_co_dcm.lua')
  assert(status_code, 'DCM failed to start')
  --
  os.exit()
elseif kind=='vision' then
  unix.chdir(HOME..'/Player')
  -- Kill old stuff
  local ps = {}
  local imu_ps = io.popen("pgrep -f state_wizard"):lines()
  for p in imu_ps do table.insert(ps, p) end
  local dcm_ps = io.popen("pgrep -f camera_wizard"):lines()
  for p in dcm_ps do table.insert(ps, p) end
  local dcm_ps = io.popen("pgrep -f world_wizard"):lines()
  for p in dcm_ps do table.insert(ps, p) end
  local dcm_ps = io.popen("pgrep -f gc_wizard"):lines()
  for p in dcm_ps do table.insert(ps, p) end
  if #ps>0 then
    print("Killing Vision processes...")
    local kill_cmd = 'kill '..table.concat(ps,' ')
    status_code = os.execute(kill_cmd)
  end
  print('Starting state wizard')
  os.execute('screen -S state -L -dm luajit state_wizard.lua')

  print('Starting camera wizard')
  os.execute('screen -S camera -L -dm luajit camera_wizard.lua')

  print('Starting world wizard')
  os.execute('screen -S world -L -dm luajit world_wizard.lua')

  print('Starting game control wizard')
  os.execute('screen -S gc -L -dm luajit gc_wizard.lua')
end

unix.usleep(1e6)
print("\n==== CHECKING STATUS ===")

dofile'check_game.lua'
