module(..., package.seeall);

CommWired=require('Comm');
-- Only send items from shared memory
require('vcm')
require('serialization');
require('Config');

--sendShm = {'wcm','vcm','gcm'}

-- Initiate Sending Address
IP = '192.168.123.201'
CommWired.init(IP,111111);
print('Receiving from',IP);

-- Add a little delay between packet sending
-- pktDelay = 500; -- time in us
-- Empirical value for keeping all packets intact
pktDelay = 1E6 * 0.001; --For image and colortable
pktDelay2 = 1E6 * 0.001; --For info

function update()
  if CommWired.size() > 0 then
    msg = CommWired.receive();
    obj = serialization.deserialize(msg);
    if (obj.arr) then
      print(obj.arr.name)
      if (string.find(obj.arr.name,'lut')) then
        print('receive lut');
      end
    end
  end
end
