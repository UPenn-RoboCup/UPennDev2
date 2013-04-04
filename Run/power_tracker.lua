dofile('include.lua')

----------------------------------------------------------------------
-- Motion Manager
----------------------------------------------------------------------

require('curses')
require('Platform')
require('dcm')
require('Config_devices')

local rpc_endpoint = 'tcp://127.0.0.1:12001'

local power = {}
local peak_power = {}
local peak_total_power = 0
local function update_power()
  local total_power = 0
  for i,joint in ipairs(Config_devices.joint.id) do
    local v = dcm:get_joint_velocity_sensor(joint)
    local f = dcm:get_joint_force_sensor(joint)
    power[i] = v*f
    total_power = total_power + math.abs(power[i])
    if math.abs(power[i]) > math.abs(peak_power[i]) then
      peak_power[i] = power[i]
    end
  end
  if total_power > peak_total_power then
    peak_total_power = total_power
  end
end

local function draw_screen()
  local total_power = 0
  curses.clear()
  curses.printw('rate : %7.2f                  Power Tracker\n', 
                 Platform.get_update_rate())
  curses.printw('///////////////////////////////////////')
  curses.printw('///////////////////////////////////////\n')
  curses.printw('                         %20s %20s\n', 'power', 'peak power')
  curses.printw('---------------------------------------')
  curses.printw('---------------------------------------\n')
  for i,joint in ipairs(Config_devices.joint.id) do
    curses.printw('%16s         %20f %20f\n', joint, power[i], peak_power[i])
    total_power = total_power + power[i]
  end
  curses.printw('---------------------------------------')
  curses.printw('---------------------------------------\n')
  curses.printw('                         %20f %20f\n', total_power, peak_total_power)
  curses.refresh()
end

-- initialize screen
curses.initscr()
curses.cbreak()
curses.noecho()
curses.keypad(1)
curses.timeout(1)

-- initialize peak_power
for i,v in ipairs(Config_devices.joint.id) do
  peak_power[i] = 0
end

local count = 0
while true do
  local key = curses.getch()
  if (key == string.byte('q')) then
    break
  end
  
  update_power()
  
  -- update screen
  if (key or (count % 20 == 0)) then
    if key == curses.KEY_RESIZE then
      local y, x = curses.getmaxyx()
      curses.resizeterm(y, x)
    end
    draw_screen()
  end
  count = count + 1
end

curses.endwin()
