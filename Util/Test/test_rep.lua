dofile'../../include.lua'
local simple_ipc = require'simple_ipc'
local rep = simple_ipc.new_replier'test'
local reply = 'world'

while true do
  local request, has_more = rep:receive()
  print('Request: ', request, has_more)
  if has_more then
    request2, has_more = rep:receive()
    print('Request2: ', request2, has_more)
  end

  print('Replying',reply)
  local ret = rep:send(reply)
  print('Return',ret)
end