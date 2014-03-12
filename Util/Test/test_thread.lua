dofile'../../include.lua'
local simple_ipc = require'simple_ipc'
local util = require'util'


-- Grab the arguments from require
local _NAME, CTX, ch_name = ...
print('... is ', _NAME, CTX, ch_name)

local function main()
	-- main thread
	local thread, ch = simple_ipc.new_thread('test_thread.lua','#test')
	util.ptable(ch)
	thread:start()
	-- Give the thread some time to start
	unix.usleep(1e5)
	
	while thread:alive() do
		print('MAIN | sending data...')
		local ret = ch:send('Hello')
		print('MAIN | ret',ret)
		local data, has_more = ch:receive()
		print('MAIN | got',data,has_more)
	end
end

local function child()
	simple_ipc.import_context( CTX )
	print("yay - I'm a thread!")
	local ch = simple_ipc.new_pair(ch_name)
	util.ptable(ch)

	for i=1,4 do
		print('THREAD waiting for data')
		local data, has_more = ch:receive()
		print('thread received',data,has_more)
		-- Simulate computation
		unix.usleep(1e6)
		-- Send the response
		ch:send('world')
	end
end

-- Run the code based on being a parent or child
if not CTX then
	main()
else
	child()
end
