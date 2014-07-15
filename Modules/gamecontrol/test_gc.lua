--require'init'
local GameController = require('GameControlReceiver')
local pktNum = 0
while true do 
  packet = GameController.receive()
  if packet and packet.packetNumber>pktNum then
    pktNum = packet.packetNumber
    for k,v in pairs(packet) do print(k,v) end
  end
--  os.execute('sleep 1')
end
