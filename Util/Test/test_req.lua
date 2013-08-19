dofile'../../include.lua'
local simple_ipc = require'simple_ipc'
local req = simple_ipc.new_requester'test'
local request = 'hello'

for count=1,100 do
  
  print('Requesting',request)
  local ret = req:send(request..count)
  print('Return',ret)

  local reply,has_more = req:receive()
  print('Reply: ',reply,has_more)

end