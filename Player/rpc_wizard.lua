dofile'include.lua'
local mp = require'msgpack'
local simple_ipc = require'simple_ipc'
local rep = simple_ipc.new_replier'test'

-- TODO: Require all necessary modules
require'vcm'
-- TODO: perform each request inside of a coroutine, so that the main thread does not die

while true do
	repeat
		request, has_more = rep:receive()
		--print('Request: ', request, has_more)
    reply = assert(loadstring(request))()
	until not has_more

	print('Replying',reply)
	local ret = rep:send( mp.pack(reply) )
	--print('Return',ret)
end
