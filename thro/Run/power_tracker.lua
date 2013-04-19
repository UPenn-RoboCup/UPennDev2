dofile('include.lua')

----------------------------------------------------------------------
-- Power Tracking
----------------------------------------------------------------------

-- Bryce says that the ball screw is ~95% efficient
--                 the belt is       ~90% efficient
-- Maxon says their motors are        87% max efficency (part #309758)
-- Total: .95 *.90 *.87 = 0.74385 = ~74% efficiency at the end of the piston

-- The non-knee motors are allegedly Maxon 309758's in the 2008 catalog;
-- the knee motors are likewise 305015's.
local maxon309758 = {
                      torque_constant_Nm_per_A=9580,
                      resistance=.836,
                      efficiency=.87,
                    }
local maxon305015 = {
                      torque_constant_Nm_per_A=27600,
                      resistance=.386,
                      efficiency=.89,
                    }

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

local torque = {}
local input_power = {}
local output_power = {}
local peak_input_power = {}
local peak_output_power = {}
local total_input_power = 0
local total_output_power = 0
local peak_total_input_power = 0
local peak_total_output_power = 0
local function update_output_power()
  total_output_power = 0
  for i,joint in ipairs(Config_devices.joint.id) do
    local v = dcm:get_joint_velocity_sensor(joint)
    local f = dcm:get_joint_force_sensor(joint)
    torque[i] = f
    output_power[i] = v*f
    total_output_power = total_output_power + math.abs(output_power[i])
    if math.abs(output_power[i]) > math.abs(peak_output_power[i]) then
      peak_output_power[i] = output_power[i]
    end
  end
  
  if total_output_power > peak_total_output_power then
    peak_total_output_power = total_output_power
  end
end

local function motor_power(motor, torque)
  return torque^2/motor.torque_constant_Nm_per_A^2*motor.resistance
end

local function update_input_power()
  total_input_power = 0
  for i,joint in ipairs(Config_devices.joint.id) do
    input_power[i] = motor_power(maxon309758, torque[i])
    -- only add the output power if it's positive
    if output_power[i] > 0 then
      input_power[i] = input_power[i] + output_power[i]
    end
    if math.abs(input_power[i]) > math.abs(peak_input_power[i]) then
      peak_input_power[i] = input_power[i]
    end
    total_input_power = total_input_power + input_power[i]
  end
  if total_input_power > peak_total_input_power then
    peak_total_input_power = total_input_power
  end
end

local elapsed_time = 0
local total_input_energy = 0
local average_input_power = 0
local total_output_energy = 0
local average_output_power = 0
local function update_totals(dt)
  elapsed_time = elapsed_time + dt
  total_input_energy = total_input_energy + total_input_power*dt
  average_input_power = total_input_energy / elapsed_time
  total_output_energy = total_output_energy + total_output_power*dt
  average_output_power = total_output_energy / elapsed_time
end


local function draw_screen()
  curses.clear()
  curses.printw('rate : %7.2f                  Power Tracker\n', 
                 Platform.get_update_rate())
  curses.printw('///////////////////////////////////////')
  curses.printw('///////////////////////////////////////\n')
  curses.printw('                   %13s %13s %13s %13s\n', 'input', 'peak input', 'output', 'peak output')
  curses.printw('---------------------------------------')
  curses.printw('---------------------------------------\n')
  for i,joint in ipairs(Config_devices.joint.id) do
    curses.printw('%16s   %13f %13f %13f %13f\n', joint, input_power[i], peak_input_power[i], output_power[i], peak_output_power[i])
  end
  curses.printw('---------------------------------------')
  curses.printw('---------------------------------------\n')
  curses.printw('%16s   %13f %13f %13f %13f\n', 'total (W)', total_input_power, peak_total_input_power, total_output_power, peak_total_output_power)
  curses.printw('%16s   %13s %13f %13s %13f\n', 'average (W)', '', average_input_power, '', average_output_power)
  curses.printw('%16s   %13s %13f %13s %13f\n', 'energy (J)', '', total_input_energy, '', total_output_energy)
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
  torque[i] = 0
  input_power[i] = 0
  output_power[i] = 0
  peak_input_power[i] = 0
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
    update_output_power()
    update_input_power()
    update_totals(time - last_time)
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
