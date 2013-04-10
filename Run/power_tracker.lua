dofile('include.lua')

----------------------------------------------------------------------
-- Power Tracking
----------------------------------------------------------------------

require('curses')
require('Platform')
require('dcm')
require('Config_devices')

local function create_joint_data_ssv(filename)
  local file = io.open(filename, 'w')
  for i,joint in ipairs(Config_devices.joint.id) do
    file:write(joint..' ')
  end
  file:write('\n')
  return file
end

local function write_joint_data_ssv(ssv_file, table)
  for i,joint in ipairs(Config_devices.joint.id) do
    ssv_file:write(table[i]..' ')
  end
  ssv_file:write('\n')
end

local power = {}
local peak_power = {}
local total_power = 0
local peak_total_power = 0
local total_energy = 0
local function update_power(dt)
  total_power = 0
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
  
  total_energy = total_energy + total_power*dt
end

local function draw_screen()
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
  curses.printw('%16s            %38f\n', 'energy (J)', total_energy)
  curses.refresh()
end

-- initialize screen
curses.initscr()
curses.cbreak()
curses.noecho()
curses.keypad(1)
curses.timeout(1)

Platform.set_update_rate(500)
Platform.entry()

-- initialize power
for i,v in ipairs(Config_devices.joint.id) do
  power[i] = 0
  peak_power[i] = 0
end

local power_ssv = create_joint_data_ssv('joint_power.ssv')

local count = 0
local last_time = Platform.get_time()
while true do
  local key = curses.getch()
  if (key == string.byte('q')) then
    break
  end
  
  local time = Platform.get_time()
  if time > last_time then
    update_power(time - last_time)
    write_joint_data_ssv(power_ssv, power)
  end
  last_time = time
  
  Platform.update()
  
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

power_ssv:close()
Platform.exit()
curses.endwin()
