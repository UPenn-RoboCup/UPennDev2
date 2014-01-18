print('\n\t===== UDP Test =====')
local udp = require 'udp'

-- Default: send and receive locally at 54321
-- -s [xxxxx] sending [to specified port]
-- -r [xxxxx] receiving [at specified port]
-- -both [][] send and receive

local udp_sender, udp_receiver
-- Default params
--local send_ip = '192.168.123.7'
local send_ip = '192.168.123.23'
local dport = 54321

-- Only send to a remote host
if arg[1] == '-s' then
	local port = arg[2] or dport
  udp_sender = udp.new_sender(send_ip, port)
-- Only receive from a remote host
elseif arg[1] == '-r' then
  local port = arg[2] or dport
  udp_receiver = udp.new_receiver(port)
-- Both send to and receive from a remote host
elseif arg[1] == '-both' then
  local send_port = arg[2] or dport
  local recv_port = arg[3] or dport
  udp_sender = udp.new_sender(send_ip, send_port)
  udp_receiver = udp.new_receiver(recv_port)
-- Send and receive locally
else
	udp_sender = udp.new_sender('127.0.0.1', dport)
  udp_receiver = udp.new_receiver(dport)
end

local msg = 'hello'
if udp_sender then
  assert(udp_sender,"Bad udp sender!")
end
if udp_receiver then
  assert(udp_receiver,"Bad udp receiver!")
end
print(
string.format("\t%s\nrecv_fd\n\t%s\n",tostring(udp_sender),tostring(udp_receiver) )
)

package.cpath = '../unix/?.so;'..package.cpath
require'unix'
--for i=1,10 do
while true do
	unix.sleep(1)  -- sec
	if udp_sender then
	  local ret = udp_sender:send( msg )
    if ret==#msg then
      print('Sent '..ret..' bytes out of '..#msg)
    else
      print('!!! LOCAL |  Sent '..ret..' bytes out of '..#msg..' !!!')
    end
	end

	while udp_receiver and udp_receiver:size()>0 do
		local data = udp_receiver:receive()
		print(string.format('\tReceived %d bytes:',#data), data )
	end
end

if udp_sender then
  udp_sender:close()
end

if udp_receiver then
  udp_receiver:close()
end
