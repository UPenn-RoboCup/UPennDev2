local two_port = true
local udp1 = require 'udp'
local msg1 = 'hello';
udp1.init('127.0.0.255',54321)
if two_port then
  udp2 = require 'udp'
  msg2 = 'world!'
  udp2.init('127.0.0.255',54322)
end
for i=1,4 do
  local ret1 = udp1.send(msg1,#msg1)
  print('1 Sent',ret1,'bytes')
  if two_port then  
    local ret2 = udp2.send(msg2,#msg2)
    print('2 Sent',ret2,'bytes')
  end
  local data1 = udp1.receive()
  print('1 received',data)
  local data2 = udp2.receive()
  print('2 received',data2)
  print()
end
