dofile'../../include.lua'
local simple_ipc = require'simple_ipc'
local rep = simple_ipc.new_replier'test'
local reply = 'world'

while true do
  local reply, has_more = rep:receive()
  print('Request: ', reply, has_more)

  print('Replying',reply)
  local ret = rep:send(reply)
  print('Return',ret)
end