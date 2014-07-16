--require'init'
local GameController = require('GameControlReceiver')
local pktNum = 0
while true do 
  packet = GameController.receive()
  if packet and packet.packetNumber>pktNum then
    pktNum = packet.packetNumber
    for k,v in pairs(packet) do 
			print(k,v) 
			if k=='teams' then
				print('coach1:', packet.teams[1].teamNumber, packet.teams[1].coachMessage)
				print('coach2:', packet.teams[2].teamNumber, packet.teams[2].coachMessage)
			end
		end
  end
--  os.execute('sleep 1')
end
