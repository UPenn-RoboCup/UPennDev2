print('\n\t== UDP Test ==')
-- Send data to MATLAB
local two_port = true

local msg = 'hello';
local udp = require 'udp'
local udp_sender = udp.new_sender('192.168.123.200',54321)
assert(udp_sender>0,"Bad udp sender!")
local udp_receiver = udp.new_receiver(54321)
assert(udp_receiver>0,"Bad udp receiver!")
print(
string.format("LOCAL |  send_fd(%d), recv_fd(%d)",udp_sender,udp_receiver)
)
--[[
while true do
  while udp.size()>0 do
    local data = udp.receive()
    print('\tLOCAL | Received',data)
  end
end
--]]

for i = 1, 10 do
  local msgg = msg..i
	local ret = udp.send(udp_sender,msgg)
	if(ret==#msgg) then
		print('LOCAL |  Sent '..ret..' bytes out of '..#msgg)
	else
		print('!!! LOCAL |  Sent '..ret..' bytes out of '..#msg..' !!!')
	end
end

--[[
if two_port then
	msg2 = 'world';
	print('Setting up MATLAB udp...')
	udp_sender_matlab = udp.new_sender('127.0.0.1',54320)
	assert(udp_sender_matlab>0,"Bad matlab udp setup!")
	print(string.format("MATLAB | send_fd(%d)",udp_sender_matlab) )
end

for i=1,4 do
	print()
	local ret = udp.send(udp_sender,msg)
	if(ret==#msg) then
		print('LOCAL |  Sent '..ret..' bytes out of '..#msg)
	else
		print('!!! LOCAL |  Sent '..ret..' bytes out of '..#msg..' !!!')
	end
	while udp.size()>0 do
		local data = udp.receive()
		print('\tLOCAL | Received',data)
	end
  
	if udp_sender_matlab then
		print()
		local ret2 = udp.send(udp_sender_matlab, msg2, #msg2)
		if(ret2==#msg2) then
			print('MATLAB | Sent '..ret2..' bytes out of '..#msg2)
		else
			print('!!! MATLAB | Sent '..ret2..' bytes out of '..#msg2..' !!!')
		end
	end
end
--]]
udp.close(udp_sender)
udp.close(udp_receiver)
if udp_sender_matlab then
	udp.close(udp_sender_matlab)
end
