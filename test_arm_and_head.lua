dofile'include.lua'
local Body = require'Body'
local getch = require'getch'
local util = require'util'
local color = util.color
local running = true
local tasks = {}

local is_red = Body.get_head_led_red()[1]
function tasks.r()
	is_red = 255 - is_red
	Body.set_head_led_red( is_red )
	return is_red>0 and "RED ON" or "RED OFF"
end

local is_blue = Body.get_head_led_blue()[1]
function tasks.b()
	is_blue = 255 - is_blue
	Body.set_head_led_blue(is_blue)
	return is_blue>0 and "BLUE ON" or "BLUE OFF"
end

local active_arm = 'left'
function tasks.a()
	active_arm = active_arm=='left' and 'right' or 'left'
end

function tasks.g()
	running = false
	return "Done"
end

local function process_kb()
	local keycode = getch.block()
	if not keycode then return end
	local char = string.char(keycode)
	local char_lower = string.lower(char)
	local func = tasks[char]
	if func then return func() end
	local func = tasks[char_lower]
	if func then return func() end
end

local function print_menu(msg)
	os.execute("clear")
	local menu_tbl = {
		color("Head LED Red", 'red')..": Press r",
		color("Head LED Green", 'green')..": Press g",
		color("Head LED Blue", 'blue')..": Press b",
		color("Active arm: ", 'yellow')..active_arm.." Press a to swap",
		"Quit: g",
		msg or '',
		'',
		"LArm: "..tostring(Body.get_larm_position()),
		"RArm: "..tostring(Body.get_rarm_position()),
	}
	print(table.concat(menu_tbl, '\n'))
end

local msg
print_menu()
while running do
	print_menu( process_kb() )
end
