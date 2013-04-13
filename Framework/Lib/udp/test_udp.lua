-- TODO: Support multiple udps
local two_port = false

local msg = 'hello';
local udp = require 'udp'
local udp_ok = udp.init('127.0.0.1',54321)
assert(udp_ok,"Bad udp setup!")

if two_port then
  msg2 = 'world';
  print('Setting up udp2')
  udp2 = require 'udp'
  udp2_ok = udp2.init('127.0.0.1',54322)
  assert(udp2_ok,"Bad udp2 setup!")
end
--for k,v in pairs(getmetatable(udp)) do print(k,v) end

for i=1,4 do
  print()
  local ret = udp.send(msg)
  print('1 Sent',ret,'bytes of',#msg)
  while udp.size()>0 do
    local data = udp.receive()
    print('1 received',data)
  end
  
  if two_port and udp2_ok then
    ret = udp.send(msg2,#msg2)
    print('2 Sent',ret,'bytes')
    while udp2.size()>0 do
      local data = udp2.receive()
      print('2 received',data)
    end
  end
end
