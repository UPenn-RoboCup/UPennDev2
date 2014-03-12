dofile'../../include.lua'
local util = require'util'
local simple_ipc = require 'simple_ipc'
--local test_channel = simple_ipc.new_subscriber('test')
local test_channel = simple_ipc.new_subscriber(55555,'localhost')

util.ptable(test_channel)

while true do
  local x = test_channel:receive()
  print('Got',#x)
end
