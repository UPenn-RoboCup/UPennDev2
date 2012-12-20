module(... or '', package.seeall)

-- Add the required paths
cwd = '.';

uname  = io.popen('uname -s')
system = uname:read();

package.cpath = cwd.."/?.so;"..package.cpath;
package.cpath = cwd.."/../../Player/Lib/?.so;"..package.cpath;
package.path = cwd.."/../../Player/Util/?.lua;"..package.path;
package.path = cwd.."/../../Player/Config/?.lua;"..package.path;
package.path = cwd.."/../../Player/Vision/?.lua;"..package.path;

require('serialization');
require('signal')
require('Serial');
require('kBPacket');
require('unix');
require('rcm');

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

-- Create files
imufilecnt = 0;
filetime = os.date('%m.%d.%Y.%H.%M');
filename = string.format("imu%s-%d", filetime, imufilecnt);

file = io.open(filename, "w");
linecount = 0;
maxlinecount = 5000;

cnti = 0;
t0 = unix.time();
t2 = unix.time();
while (1) do
  t1 = unix.time();
  packet, size = ReceivePacket();
  if (type(packet) == 'userdata') then
    cnti = cnti + 1;
    print (1/(unix.time()-t0));
    print(size)
    t0 = unix.time(); -- timestamp
    
    width = 33;
    height = 1;
    IMUArray = serialization.serialize_array(packet, width,
                height, 'uint8', 'imu', cnti);
    saveIMU = {};
    saveIMU.timestamp = t1;
    saveIMU.arr = IMUArray[1];
    local savedata=serialization.serialize(saveIMU);
    print(savedata);
--    savedata = Z.compress(savedata, #savedata);
    file:write(savedata);
    file:write('\n');
    linecount = linecount + 1;

  end

  if linecount > maxlinecount then
    linecount = 0;
    file:close();
    imufilecnt = imufilecnt + 1;
    filename = string.format("imu%s-%d", filetime, imufilecnt);
    file = io.open(filename, "w");
  end
end

file:close();
