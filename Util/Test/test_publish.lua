dofile'../../include.lua'
local util = require'util'
local simple_ipc = require 'simple_ipc'
require'unix'
--local test_channel = simple_ipc.new_publisher('test') --ipc
local test_channel = simple_ipc.new_publisher(43210) --tcp
util.ptable(test_channel)
local cnt = 1
while true do
	local msg = table.concat({
		'#', cnt, ': ', math.random(9)
	})
  test_channel:send(msg)
	print(msg)
  unix.usleep(5e5)
	cnt = cnt + 1
end
