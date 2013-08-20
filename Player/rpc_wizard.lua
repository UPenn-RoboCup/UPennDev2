dofile'include.lua'
local mp = require'msgpack'
local simple_ipc = require'simple_ipc'
local rep = simple_ipc.new_replier'test'
local util = require'util'

-- TODO: Require all necessary modules
require'vcm'

while true do
	repeat
		request, has_more = rep:receive()
    local rpc_tbl = mp.unpack(request)
		
    --util.ptable(rpc_tbl)
    
    method = rpc_tbl.call..'_'..rpc_tbl.segment..'_'..rpc_tbl.key
    func = _G[rpc_tbl.memory][method]
    
    --[[
    print('method',method,func)
    if type(rpc_tbl.val)=='table' then
      util.ptable(rpc_tbl.val)
    else
      print(rpc_tbl.val)
    end
    --]]
    
    -- Use a protected call so that we do not error out on a bad RPC
    status, reply = pcall(func,rpc_tbl.val)
    -- TODO: if set ensure rpc_tbl.val, and other checks
    
    --print('RPC status',status)
    
	until not has_more

	print( util.color('RPC:','yellow'), util.color(method,'green'), reply )
	local ret = rep:send( mp.pack(reply) )
	--print('Return',ret)
end
