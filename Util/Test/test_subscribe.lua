local test_poller = true
dofile'../../include.lua'
local p
local util = require'util'
local simple_ipc = require 'simple_ipc'
--local test_channel = simple_ipc.new_subscriber('test')
local test_channel = simple_ipc.new_subscriber(43210,'192.168.123.222')
test_channel.callback = function(s)
	local c = p.lut[s]
	local data, has_more = c:receive()
	print(s, 1, data, has_more)
end

print('-- test_channel')
util.ptable(test_channel)
print('--')

if test_poller then
	local test_channel2 = simple_ipc.new_subscriber('test2')
	test_channel2.callback = function(s)
		local c = p.lut[s]
		local data, has_more = c:receive()
		print(s,2, data, has_more)
	end
	p = simple_ipc.wait_on_channels{test_channel,test_channel2}
	print('Poller',p,p.lut)
	p:start()
	return
end

while true do
  local x = test_channel:receive()
  print('Got',#x, x)
end
