#!/usr/bin/env luajit
assert(arg, "Run from the shell")
--local kind = assert(arg[1], "Specify vision or motion")

dofile'include.lua'
local color = require'util'.color

local LUA = 'luajit'

local wizards = {
	{'feedback', },
	{'mesh', },
	{'slam', },
	{'rpc', },
	{'lidar_chest', },
	{'lidar_head', },
	{'camera', 1},
	{'camera', 2},
}
local runs = {
	'dcm',
	'imu'
}

unix.chdir(HOME..'/Run')

local function gen_screen(name, script, ...)
	local args = {...}
	return table.concat({
		'screen',
		'-S',
		name..table.concat(args),
		'-L',
		'-dm',
		LUA,
		name..'_wizard.lua',
		...
	},' ')
end

for i, wizard in ipairs(wizards) do
	local name = wizard[1]
	local scriptname = wizard[1]..'_wizard.lua'
	local name_w_args = wizard[2] and scriptname..' '..wizard[2] or scriptname
	-- Kill any previous instances
	local ret = io.popen("pkill -f "..name_w_args):lines()
	for pid in ret do
		print('Killed Process', pid)
	end
	local script = gen_screen(name, name..'_wizard.lua', unpack(wizard, 2))
	local status = os.execute(script)
	print(color(name, 'yellow'), 'starting')
	print(script)
end

-- Print starting the robot
if RUN_ROBOT then
	unix.chdir(ROBOT_HOME)
	for i, run in ipairs(runs) do
		-- Kill any previous instances
		local ret = io.popen("pkill -f "..run):lines()
		for i, pid in ret do
			print('Killed Process', pid)
		end
		local script = gen_screen(run, 'run_'..run..'.lua')
		local status = os.execute(script)
		print(color(run, 'yellow'), 'starting')
		--print(script)
		unix.usleep(1e6)
	end
end

-- Check the status
unix.sleep(1)
for i, wizard in ipairs(wizards) do
	local name = wizard[1]
	-- Kill any previous instances
	local ret = io.popen("pgrep -fla "..name):lines()
	local started = false
	for pid in ret do
		if pid then started = true end
	end
	if started then
		print(color(name, 'green'), 'started')
	else
		print(color(name, 'red'), 'not started')
	end
end

if RUN_ROBOT then
	unix.sleep(1)
	for i, name in ipairs(runs) do
		-- Kill any previous instances
		local ret = io.popen("pgrep -f "..name):lines()
		local started = false
		for pid in ret do
			if pid then started = true end
		end
		if started then
			print(color(name, 'green'), 'started')
		else
			print(color(name, 'red'), 'not started')
		end
	end
end
