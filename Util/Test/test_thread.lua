dofile'../../include.lua'
--print_env()
local simple_ipc = require'simple_ipc'
local util = require'util'

-- Grab the arguments from require
local CTX, metadata = ...
print('... is ', type(CTX), type(metadata) )

local function main()
	-- Setup the children
	local meta = {}
	meta.bus = 1
	meta.dev = "/dev/ttyUSB0"
	meta.name = 'chainA'
	local threadA, chA = simple_ipc.new_thread('test_thread.lua',meta.name,meta)
	chA.thread = threadA
	--util.ptable(chA)
	
	-- Setup the children
	local meta = {}
	meta.bus = 2
	meta.dev = "/dev/ttyUSB1"
	meta.name = 'chainB'
	local threadB, chB = simple_ipc.new_thread('test_thread.lua',meta.name,meta)
	chB.thread = threadB
	--util.ptable(chB)
	
	-- Poll on the threads
	local p
	local function cb(s)
		local ch = p.lut[s]
		local data, has_more = ch:receive()
		print('MAIN | Got',data,has_more)
		if ch.thread:alive() then
			ch:send('More work')
		else
			-- Cleanup
			print('**** Removing',ch.name)
			if p:clean(s)<1 then p:stop() end
			ch.thread:join()
		end
	end
	chA.callback = cb
	chB.callback = cb
	p = simple_ipc.wait_on_channels{chA,chB}
	--print('====')
	--util.ptable(p)
	--print('====')
	
	-- Give the thread some time to start
	threadA:start()
	unix.usleep(1e5)
	threadB:start()
	unix.usleep(1e5)
	
	-- Send the initial work
	chA:send('work1')
	chB:send('work2')
	
	p:start()
	print('Done!')

end

local function child(meta)
	simple_ipc.import_context( CTX )
	print("yay - I'm a thread!")
	util.ptable(metadata)
	local ch = simple_ipc.new_pair(metadata.ch_name)
	--util.ptable(ch)

	for i=1,4 do
		print('THREAD waiting for data')
		local data, has_more = ch:receive()
		print(meta.name,'received',data,has_more)
		-- Simulate computation
		unix.usleep(meta.bus*1e6)
		-- Send the response
		ch:send(meta.dev)
	end
end

-- Run the code based on being a parent or child
if CTX and type(metadata)=='table' then
	print('^^^^ SPAWN CHILD ***')
	child(metadata)
else
	main()
end
