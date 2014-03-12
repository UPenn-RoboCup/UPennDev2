dofile'../../include.lua'
local simple_ipc = require'simple_ipc'
local util = require'util'

-- Grab the arguments from require
local CTX, metadata = ...
print('... is ', CTX, metadata)

local function main()
	-- Setup the children
	local meta = {}
	meta.bus = 1
	meta.dev = "/dev/ttyUSB0"
	meta.name = 'chainA'
	local threadA, chA = simple_ipc.new_thread('test_thread.lua',meta.name,meta)
	util.ptable(chA)
	
	-- Setup the children
	local meta = {}
	meta.bus = 2
	meta.dev = "/dev/ttyUSB1"
	meta.name = 'chainB'
	local threadB, chB = simple_ipc.new_thread('test_thread.lua',meta.name,meta)
	util.ptable(chB)
	
	-- Give the thread some time to start
	threadA:start()
	unix.usleep(1e5)
	threadB:start()
	unix.usleep(1e5)
	
	while threadA:alive() do
		print('MAIN | sending data...')
		chA:send('Hello')
		chB:send('Hi')
		local dataA, has_more = chA:receive()
		print('MAIN | got',dataA,has_more)
		local dataB, has_more = chB:receive()
		print('MAIN | got',dataB,has_more)
	end
end

local function child(meta)
	simple_ipc.import_context( CTX )
	print("yay - I'm a thread!")
	util.ptable(metadata)
	local ch = simple_ipc.new_pair(metadata.ch_name)
	util.ptable(ch)

	for i=1,4 do
		print('THREAD waiting for data')
		local data, has_more = ch:receive()
		print('thread received',data,has_more)
		-- Simulate computation
		unix.usleep(1e6)
		-- Send the response
		ch:send(meta.dev)
	end
end

-- Run the code based on being a parent or child
if type(CTX)=='userdata' and type(metadata)=='table' then
	child(metadata)
else
	main()
end
