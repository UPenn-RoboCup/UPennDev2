print('\n\t== UDP Test ==')
-- Send data to MATLAB
local two_port = true

local msg = 'hello';
local udp = require 'udp'
local udp_sender = udp.new_sender('127.0.0.1',54321)
print(udp_sender)
--assert(udp_sender>0,"Bad udp sender!")
local udp_receiver = udp.new_receiver(54321)
print(udp_receiver)


if two_port then
	msg2 = 'world';
	print('Setting up MATLAB udp...')
	udp_sender_matlab = udp.new_sender('127.0.0.1',54320)
  print(udp_sender_matlab)
  udp_receiver_matlab = udp.new_receiver(54320)
  print(udp_receiver_matlab)
end

for i=1,4 do
	print()
	local ret = udp_sender:send(msg)
	if(ret==#msg) then
		print('LOCAL |  Sent '..ret..' bytes out of '..#msg)
	else
		print('!!! LOCAL |  Sent '..ret..' bytes out of '..#msg..' !!!')
	end
	while udp_receiver:size()>0 do
		local data = udp_receiver:receive()
		print('\tLOCAL | Received',data)
	end

	while udp_receiver_matlab:size()>0 do
		local data = udp_receiver_matlab:receive()
		print('\tLOCAL Matlab | Received',data)
	end
  
	if udp_sender_matlab then
		print()
		local ret2 = udp_sender_matlab:send(msg2, #msg2)
		if(ret2==#msg2) then
			print('MATLAB | Sent '..ret2..' bytes out of '..#msg2)
		else
			print('!!! MATLAB | Sent '..ret2..' bytes out of '..#msg2..' !!!')
		end
	end
end
