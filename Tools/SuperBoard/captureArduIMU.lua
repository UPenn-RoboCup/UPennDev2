local pwd = os.getenv('PWD')
package.cpath = pwd..'/../qt/lib/qt/?.so;'..package.cpath
package.path = pwd..'/../qt/lib/ffi/?.lua;'..package.path

--require 'include'
local ffi = require 'ffi'
local Serial = require('Serial');
local unix = require('unix');

fps = 150;
seconds = 20
nframes = fps*seconds;

dev = '/dev/ttyUSB0';
baud = 115200;
s1 = Serial.connect(dev, baud);

packetID = -1;
function ReceivePacket() 
  buf, buftype, bufsize = Serial.read(36, 5000);
  return buf, bufsize
end

-- Create files
function cdata2string(cdata, len)
  str = '';
  for i = 0, len - 1 do
    if cdata[i] == 40 or cdata[i] == 41 then cdata[i] = 32 end
    str = str..string.format('%c', cdata[i])
  end
  return str
end

-- Log files
file = io.open('imu', 'w')
tfile = io.open('time', 'w')

for fr=1,nframes do
  t1 = unix.time();
  packet, size = ReceivePacket();
  if size > 0 then
    data = ffi.cast('uint8_t*', packet)
    str = cdata2string(data, size)
    if str[-1] ~= '*' then -- Last character delimiter
      packet, size = ReceivePacket();
      data = ffi.cast('uint8_t*', packet)
      str = str..cdata2string(data, size)
    end
    tfile:write(unix.time()) tfile:write('\n')
    file:write(str)
  end
end

file:close();
tfile:close();
