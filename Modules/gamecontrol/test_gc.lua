--require'init'
local GameController = require('GameControlReceiver')

while true do 
        packet = GameController.receive()
        if packet and packet.state then
          print(packet.state)
        end
end
