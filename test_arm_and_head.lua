dofile'include.lua'
local Body = require'Body'
local getch = require'getch'
local util = require'util'
local vector = require'vector'
local color = util.color
local running = true
local tasks = {}
local t0 = unix.time()


local is_red = Body.get_head_led_red()[1]
function tasks.r()
	is_red = 255 - is_red
	Body.set_head_led_red( is_red )
	Body.set_lleg_led_red(is_red)
	Body.set_rleg_led_red(is_red)

	Body.set_larm_command_position({90*math.pi/180,10*math.pi/180})
	Body.set_rarm_command_position({90*math.pi/180,10*math.pi/180})


	return is_red>0 and "RED ON" or "RED OFF"
end

local is_blue = Body.get_head_led_green()[1]
function tasks.b()
	is_blue = 255 - is_blue
	Body.set_head_led_green(is_blue)
	Body.set_lleg_led_green(is_blue)
	Body.set_rleg_led_green(is_blue)
	return is_blue>0 and "BLUE ON" or "BLUE OFF"
end

local active_arm = 'larm'
function tasks.a()
	active_arm = active_arm=='larm' and 'rarm' or 'larm'
end

function tasks.g()
	running = false
	return "Done"
end

local function arm_delta(delta)
  local cur = Body['get_'..active_arm..'_command_position']()
	Body['set_'..active_arm..'_command_position'](cur + delta)
end

tasks['+'] = function()
	local delta = vector.new({5, 0}) * DEG_TO_RAD
  arm_delta(delta)
	return "Increase Roll"
end
tasks['='] = function()
	local delta = vector.new({0, 5}) * DEG_TO_RAD
  arm_delta(delta)
	return "Increase Pitch"
end

tasks['_'] = function()
	local delta = -1*vector.new({5, 0}) * DEG_TO_RAD
  arm_delta(delta)
	return "Increase Roll"
end
tasks['-'] = function()
	local delta = -1*vector.new({0, 5}) * DEG_TO_RAD
  arm_delta(delta)
	return "Increase Pitch"
end

local tq = {
  larm = Body.get_larm_torque_enable()[1],
  rarm = Body.get_rarm_torque_enable()[1],
}
function tasks.t()
	local delta = vector.new({0, 5}) * DEG_TO_RAD
  local tq = Body['get_'..active_arm..'_torque_enable']()
	tq = vector.ones(#tq) - tq
	Body['set_'..active_arm..'_torque_enable'](
		tq
	)
  tq[active_arm] = tq[1]
	return tq[1]==0 and "Torquing off..." or "Torquing on..."
end

local function process_kb()
  local keycode = getch.nonblock()
--	local keycode = getch.block()
	if not keycode then return end
	local char = string.char(keycode)
	local char_lower = string.lower(char)
	local func = tasks[char]
	if func then return func() end
	local func = tasks[char_lower]
	if func then return func() end
end

local lleg_temp = {}
local head_volt = {}
local function print_menu(msg)
	os.execute("clear")
  local t = unix.time()
  local tq_on = tq[active_arm]==0
	local menu_tbl = {
    string.format("Arm & Head %.2fs uptime", t - t0),
		color("Head LED Red", 'red')..": Press r",
--		color("Head LED Green", 'green')..": Press g",
		color("Head LED Blue", 'blue')..": Press b",
    '',
		color( active_arm=='larm' and '** Left Arm **' or '** Right Arm **' , 'yellow').." Press a to swap",
		"Torque "..(tq[active_arm]==0 and 'ON' or 'OFF').." Press t to toggle",
    "Press -/= to decrease angle",
    "Hold SHIFT and Press -/= to increase angle",
		"Quit: Press g",
    '',
    color("Voltage (V): ",'cyan')..tostring(head_volt),
    color("Temperature (C): ",'magenta')..tostring(lleg_temp),
		'',
		"LArm (deg): "..tostring(Body.get_larm_position() * RAD_TO_DEG),
		"RArm (deg): "..tostring(Body.get_rarm_position() * RAD_TO_DEG),
		"Lidar (deg): "..tostring(Body.get_lidar_position() * RAD_TO_DEG),
		'',
		msg or '',
	}
	print(table.concat(menu_tbl, '\n'))
end

local t_last_temp = 0
local msg
print_menu()
while running do
  
  local t = unix.time()
  if t - t_last_temp > 1 then
    t_last_temp = t
    lleg_temp = Body.get_lleg_temperature()
    rleg_temp = Body.get_rleg_temperature()
    head_volt = Body.get_head_voltage() / 10
  end

	print_menu( process_kb() )
  unix.usleep(1e5)
end
