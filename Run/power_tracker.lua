dofile('include.lua')

----------------------------------------------------------------------
-- Power Tracking
----------------------------------------------------------------------

require('curses')
require('Platform')
require('dcm')
require('Config_devices')
require('filter')

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

local filters = {}
local function write_filtered_joint_data_ssv(ssv_file, table)
  for i,joint in ipairs(Config_devices.joint.id) do
    ssv_file:write(filters[i]:update(table[i])..' ')
  end
  ssv_file:write('\n')
end

local output_power = {}
local peak_output_power = {}
local total_output_power = 0
local peak_total_output_power = 0
local total_energy = 0
local function update_output_power(dt)
  total_output_power = 0
  for i,joint in ipairs(Config_devices.joint.id) do
    local v = dcm:get_joint_velocity_sensor(joint)
    local f = dcm:get_joint_force_sensor(joint)
    output_power[i] = v*f
    total_output_power = total_output_power + math.abs(output_power[i])
    if math.abs(output_power[i]) > math.abs(peak_output_power[i]) then
      peak_output_power[i] = output_power[i]
    end
  end
  
  if total_output_power > peak_total_output_power then
    peak_total_output_power = total_output_power
  end
  
  total_energy = total_energy + total_output_power*dt
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
    curses.printw('%16s         %20f %20f\n', joint, output_power[i], peak_output_power[i])
    total_output_power = total_output_power + output_power[i]
  end
  curses.printw('---------------------------------------')
  curses.printw('---------------------------------------\n')
  curses.printw('                         %20f %20f\n', total_output_power, peak_total_output_power)
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

-- initialize output_power
for i,v in ipairs(Config_devices.joint.id) do
  local walking_frequency = .8
  filters[i] = filter.new_low_pass(1/Platform.get_update_rate(), walking_frequency)
  output_power[i] = 0
  peak_output_power[i] = 0
end

local output_power_filtered_ssv = create_joint_data_ssv('joint_output_power_filtered.ssv')
local output_power_ssv = create_joint_data_ssv('joint_output_power.ssv')

local count = 0
local last_time = Platform.get_time()
while true do
  local key = curses.getch()
  if (key == string.byte('q')) then
    break
  end
  
  local time = Platform.get_time()
  if time > last_time then
    update_output_power(time - last_time)
    write_filtered_joint_data_ssv(output_power_filtered_ssv, output_power)
    write_joint_data_ssv(output_power_ssv, output_power)
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

output_power_filtered_ssv:close()
output_power_ssv:close()
Platform.exit()
curses.endwin()
