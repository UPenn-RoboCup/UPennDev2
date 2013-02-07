require 'include'

local ffi = require 'ffi'
local serialization = require('serialization');
local util = require 'util'
local Serial = require('Serial');
local kBPacket = require('kBPacket');

ffi.cdef[[
  typedef long int __time_t;
  typedef long int __suseconds_t;
  typedef struct timeval {
    __time_t tv_sec;    /* Seconds.  */
    __suseconds_t tv_usec;  /* Microseconds.  */
  };
  int gettimeofday(struct timeval *restrict tp, void *restrict tzp);
  int poll(struct pollfd *fds, unsigned long nfds, int timeout);
]]

function utime()
  local t = ffi.new('struct timeval')
  ffi.C.gettimeofday(t, nil)
  return t.tv_sec + 1e-6 * t.tv_usec
end

function usleep(s)
  ffi.C.poll(nil, 0, s * 1000)
end

baud = 230400;
dev = '/dev/ttyUSB0';
s1 = Serial.connect(dev, baud);

packetID = -1;
function ReceivePacket() 
  if packetID < 0 then
    packetID = kBPacket.create();
  end
  
  buf, buftype, bufsize = Serial.read(1000, 20000);

--  return buf, bufsize
  packet, packetType, packetSize, buf2, buf2type, buf2Size = kBPacket.processBuffer(packetID, buf, bufsize);

  return packet, packetSize;
end

function cdata2gpsstring(cdata, len)
  str = '';
  for i = 5, len - 1 - 8 do
    str = str..string.format('%c', cdata[i])
  end
  return str
end

function cdata2string(cdata, len)
  str = '';
  for i = 0, len - 1 do
    str = str..string.format('%c', cdata[i])
  end
  return str
end

function extractImu(imustr, len)
  local imu = {}
  imu.type = 'imu'

--  imustr = ffi.new("uint8_t[?]", #imustrs, imustrs)
  imu.tuc = tonumber(ffi.new("uint32_t", bit.bor(bit.lshift(imustr[8], 24),
                      bit.lshift(imustr[7], 16), bit.lshift(imustr[6], 8), imustr[5])))
  imu.id = tonumber(ffi.new("double", imustr[9]))
  imu.cntr = tonumber(ffi.new("double", imustr[10]))
  rpyGain = 5000
  imu.r =  tonumber(ffi.new('int16_t', bit.bor(bit.lshift(imustr[12], 8), imustr[11]))) / rpyGain
  imu.p =  tonumber(ffi.new('int16_t', bit.bor(bit.lshift(imustr[14], 8), imustr[13]))) / rpyGain
  imu.y =  tonumber(ffi.new('int16_t', bit.bor(bit.lshift(imustr[16], 8), imustr[15]))) / rpyGain
  wrpyGain = 500
  imu.wr = tonumber(ffi.new("int16_t", bit.bor(bit.lshift(imustr[18], 8), imustr[17]))) / wrpyGain
  imu.wp = tonumber(ffi.new("int16_t", bit.bor(bit.lshift(imustr[20], 8), imustr[19]))) / wrpyGain
  imu.wy = tonumber(ffi.new("int16_t", bit.bor(bit.lshift(imustr[22], 8), imustr[21]))) / wrpyGain
  accGain = 5000
  imu.ax = tonumber(ffi.new("int16_t", bit.bor(bit.lshift(imustr[24], 8), imustr[23]))) / accGain
  imu.ay = tonumber(ffi.new("int16_t", bit.bor(bit.lshift(imustr[26], 8), imustr[25]))) / accGain
  imu.az = tonumber(ffi.new("int16_t", bit.bor(bit.lshift(imustr[28], 8), imustr[27]))) / accGain
  return imu;
end

function extractMag(magstr, len)
  local mag = {}
  mag.type = 'mag'

  mag.id = tonumber(ffi.new("double", magstr[5]))
  mag.tuc = tonumber(ffi.new("uint32_t", bit.bor(bit.lshift(magstr[9], 24),
                    bit.lshift(magstr[8], 16), bit.lshift(magstr[7], 8), magstr[6])))
  mag.press = tonumber(ffi.new('int16_t', bit.bor(bit.lshift(magstr[11], 8), magstr[10]))) + 100000
  mag.temp =  tonumber(ffi.new('int16_t', bit.bor(bit.lshift(magstr[15], 8), magstr[14]))) / 100
  mag.x = tonumber(ffi.new("int16_t", bit.bor(bit.lshift(magstr[19], 8), magstr[18])))
  mag.y = tonumber(ffi.new("int16_t", bit.bor(bit.lshift(magstr[21], 8), magstr[20])))
  mag.z = tonumber(ffi.new("int16_t", bit.bor(bit.lshift(magstr[23], 8), magstr[22])))
  return mag;
end

function extractGPS(gpsstr, len)
  local gps = {}
  gps.type = 'gps'
  gps.line = str
  return gps
end


function pcdata(cdata, size)
  str = ''
  for i = 0, size - 1 do
    str = str..' '..cdata[i]
  end
  print(str)
end

-- Create files
filecnt = 0;
filetime = utime();
filename = string.format("log-%s-%d", filetime, filecnt);

file = io.open(filename, "w");
linecount = 0;
maxlinecount = 500;

while (1) do
  packet, size = ReceivePacket();
  if (type(packet) == 'userdata') then
    local rawdata = ffi.cast('uint8_t*', packet)
    local timestamp = utime()
    local data = nil
    if rawdata[2] == 0 then
      if rawdata[4] == 31 then
        str = cdata2gpsstring(rawdata, size)
        data = extractGPS(str, #str)
        data.timestamp = timestamp
      elseif rawdata[4] == 34 then
        data = extractImu(rawdata, size)
        data.timestamp = timestamp
      elseif rawdata[4] == 35 then
        data = extractMag(rawdata, size)
        data.timestamp = timestamp
      end
      if data then
        savedata = serialization.serialize(data)
        file:write(savedata)
        file:write('\n')
        print(linecount, savedata)
        linecount = linecount + 1
      end
    end
  end
  if linecount >= maxlinecount then
    linecount = 0;
    file:close();
    filecnt = filecnt + 1;
    filename = string.format("log-%s-%d", filetime, filecnt);
    file = io.open(filename, "w");
  end
end

file:close();
