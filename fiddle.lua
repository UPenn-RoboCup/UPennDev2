#!/usr/local/bin/luajit -i

if IS_FIDDLE then return end
local ok = pcall(dofile, '../include.lua')
if not ok then pcall(dofile, 'include.lua') end

-- Important libraries in the global space
mp = require'msgpack.MessagePack'
si = require'simple_ipc'
local libs = {
	'ffi',
	'torch',
	'util',
	'vector',
	'Body',
}
-- Load the libraries
for _,lib in ipairs(libs) do
	local ok, lib_tbl = pcall(require, lib)
	if ok then
		_G[lib] = lib_tbl
	else
		print("Failed to load", lib)
		print(lib_tbl)
	end
end

-- FSM communicationg
fsm_chs = {}
for sm, en in pairs(Config.fsm.enabled) do
	local fsm_name = sm..'FSM'
	local ch = en and si.new_publisher(fsm_name.."!") or si.new_dummy()
	_G[sm:lower()..'_ch'] = ch
	fsm_chs[fsm_name] = ch
end

-- Shared memory
local listing = unix.readdir(HOME..'/Memory')
local shm_vars = {}
for _,mem in ipairs(listing) do
	local found, found_end = mem:find'cm'
	if found then
		local name = mem:sub(1,found_end)
		table.insert(shm_vars,name)
		require(name)
	end
end

-- Local RPC for testing
rpc_ch = si.new_requester'rpc'
dcm_ch = si.new_publisher'dcm!'
state_ch = si.new_publisher'state!'

--print(util.color('FSM Channel', 'yellow'), table.concat(fsm_chs, ' '))
--print(util.color('SHM access', 'blue'), table.concat(shm_vars,  ' '))

local function gen_screen(name, script, ...)
	return table.concat({
			'screen',
			'-S',
			name,
			'-L',
			'-dm',
			'luajit',
			script,
			...
		},' ')
end
local function kill(name)
	local ret = io.popen("pkill -f "..name)
	--for pid in ret:lines() do print('Killed Process', pid) end
end

-- Start script
local runnable = {}
for _,fname in ipairs(unix.readdir(HOME..'/Run')) do
	local found, found_end = fname:find'_wizard'
	if found then
		local name = fname:sub(1,found-1)
		runnable[name] = 'wizard'
	end
end
for _,fname in ipairs(unix.readdir(ROBOT_HOME)) do
	local found, found_end = fname:find'run_'
	local foundlua, foundlua_end = fname:find'.lua'
	if found and foundlua then
		local name = fname:sub(found_end+1, foundlua-1)
		runnable[name] = 'robot'
	end
end
function sstart(scriptname, ...)
	local kind = runnable[scriptname]
	if not kind then return false end
	local script = kind=='wizard' and scriptname..'_wizard.lua' or 'run_'..scriptname..'.lua'
	kill(script)
	unix.chdir(kind=='wizard' and HOME..'/Run' or ROBOT_HOME)
	local screen = gen_screen(scriptname, script, ...)
	print('screen', screen)
	local status = os.execute(screen)
	unix.usleep(1e6/4)
	local ret = io.popen("pgrep -fla "..script)
	for pid in ret:lines() do
		if pid:find('luajit') then
			return true
		end
	end
end

IS_FIDDLE = true

if arg and arg[-1]=='-i' and jit then
	if arg[1] then
		-- Test file first
		dofile(arg[1])
	end
	-- Interactive LuaJIT
	package.path = package.path..';'..HOME..'/Tools/iluajit/?.lua'
	dofile(HOME..'/Tools/iluajit/iluajit.lua')
end
