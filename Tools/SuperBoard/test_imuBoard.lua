module(... or '', package.seeall)

-- Add the required paths
cwd = '.';

uname  = io.popen('uname -s')
system = uname:read();

package.cpath = cwd.."/?.so;"..package.cpath;
package.cpath = cwd.."/../../../UPennDev/Player/Lib/?.so;"..package.cpath;
package.path = cwd.."/../../../UPennDev/Player/Util/?.lua;"..package.path;
package.path = cwd.."/../../../UPennDev/Player/Config/?.lua;"..package.path;
package.path = cwd.."/../../../UPennDev/Player/Vision/?.lua;"..package.path;

require('serialization');
require('Hokuyo')
require('signal')
require('Serial');
require('kBPacket');
require('unix');
require('rcm');

hokuyo = {}
hokuyo.serial = "00805676"
--hokuyo.serial = "00907258"
hokuyo.device = "/dev/ttyACM0"
Hokuyo.open(hokuyo.device, hokuyo.serial);


dev = '/dev/ttyUSB0';
baud = 230400;
s1 = Serial.connect(dev, baud);
--
packetID = -1;
function ReceivePacket() 
  if packetID < 0 then
    packetID = kBPacket.create();
  end
  
  buf, buftype, bufsize = Serial.read(1000, 2000);
--  print(type(buf))

  packet, packetType, packetSize, buf2, buf2type, buf2Size = kBPacket.processBuffer(packetID, buf, bufsize);
  return packet, packetSize;

end

function ShutDownFN()
  print("Proper shutdown")
  Hokuyo.shutdown()
  os.exit(1);
end

cntr = 0;
cnti = 0;
t0 = unix.time();
t2 = unix.time();
while (1) do
  t1 = unix.time(); -- timestamp

  packet, size = ReceivePacket();
  if (type(packet) == 'userdata') then
    cnti = cnti + 1;
    print(size);
     
  end
  if (cnti % 40 == 0) then
    print("IMU Board rate "..40/(unix.time() - t2));
    t2 = unix.time();
  end


  Hokuyo.update();
  cntr = cntr + 1;
  if (cntr % 40 == 0) then
    print("Scan rate "..40/(unix.time() - t0));
    t0 = unix.time();
  end

  lidar = Hokuyo.retrieve();
  width = rcm.nReturns;
  height = 1;
  lidarArray = serialization.serialize_array(lidar.ranges, width,
                height, 'single', 'ranges', lidar.counter);
  savelidar = {};
  savelidar.timestamp = t1;
  savelidar.arr = lidarArray[1];
  local savedata=serialization.serialize(savelidar);
  savedata = Z.compress(savedata, #savedata);


  signal.signal("SIGINT", ShutDownFN);
  signal.signal("SIGTERM", ShutDownFN);

end
