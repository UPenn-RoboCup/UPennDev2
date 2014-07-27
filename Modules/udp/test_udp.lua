print('\n\t== UDP Test ==')
-- Send data to MATLAB
local two_port = false

local msg = 'hello';
local udp = require 'udp'
local udp_sender = udp.new_sender('127.0.0.1',54321)
assert(udp_sender,"Bad udp sender!")
local udp_receiver = udp.new_receiver(54321)
assert(udp_receiver,"Bad udp receiver!")
print(
string.format("LOCAL | \nsend_fd:\n\t%s\nrecv_fd\n\t%s\n",tostring(udp_sender),tostring(udp_receiver) )
)

if two_port then
	msg2 = 'world';
	print('Setting up MATLAB udp...')
	udp_sender_matlab = udp.new_sender('127.0.0.1',54320)
	assert(udp_sender_matlab,"Bad matlab udp setup!")
	print(string.format("MATLAB | send_fd:\n\t%s\n", tostring(udp_sender_matlab)) )
end

for i=1,4 do
	local ret = udp_sender:send( msg )
	if ret==#msg then
		print('LOCAL |  Sent '..ret..' bytes out of '..#msg)
	else
		print('!!! LOCAL |  Sent '..ret..' bytes out of '..#msg..' !!!')
	end
  local data = udp_receiver:receive()
	while data do
		print(string.format('\tLOCAL | Received %d bytes:',#data),data )
    data = udp_receiver:receive()
	end

	if udp_sender_matlab then
		print()
		local ret2,error_msg2 = udp_sender_matlab:send( msg2, #msg2 )
		if ret2==#msg2 then
			print('MATLAB | Sent '..ret2..' bytes out of '..#msg2)
		else
			print('!!! MATLAB | Sent '..ret2..' bytes out of '..#msg2..' !!!',error_msg2)

		end
	end
end

udp_sender:close()
udp_receiver:close()
if udp_sender_matlab then
	udp_sender_matlab:close()
end
