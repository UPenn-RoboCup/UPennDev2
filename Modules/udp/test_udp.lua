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
local unix = require'unix'

local tbl = {}
for i=1,10000 do
	tbl[i] = i
end
msg = table.concat(tbl,',')

if two_port then
	msg2 = 'world';
	print('Setting up MATLAB udp...')
	udp_sender_matlab = udp.new_sender('127.0.0.1',54320)
	assert(udp_sender_matlab,"Bad matlab udp setup!")
	print(string.format("MATLAB | send_fd:\n\t%s\n", tostring(udp_sender_matlab)) )
end

for i=1,100 do
	local ret, uuid = udp_sender:send_all( msg )
	if not uuid and ret==#msg then
		io.write('LOCAL | Sent ', ret, ' bytes of ', #msg, '\n')
	elseif uuid then
		io.write(uuid,' | Sent ', #msg, ' bytes in ', #ret, ' packets: ', table.concat(ret, ', '), '\n')
	else
		print('!!! LOCAL |  Sent '..ret..' bytes out of '..#msg..' !!!')
	end
	local tbl_recv = {}
	while udp_receiver:size()<1 do unix.usleep(1e5) end
	local full_pkt = udp_receiver:receive()
	--io.write('\tLOCAL | Received ', #full_pkt, ' bytes', '\n')
	--print(full_pkt)
	local f = full_pkt:gmatch('%d+')
	for n in f do table.insert(tbl_recv, tonumber(n)) end
	for ii, v in ipairs(tbl_recv) do
		assert(v==tbl[ii], v..'->'..tbl[ii]..'@'..ii)
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

print('GOOD')

udp_sender:close()
udp_receiver:close()
if udp_sender_matlab then
	udp_sender_matlab:close()
end
print('DONE')