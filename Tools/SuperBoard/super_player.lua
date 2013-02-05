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
require('util');
require('unix');
require('cutil');
require('Serial');
require('rcm');
require('vector');

function parse_name(namestr)
  name = {}
  name.str = string.sub(namestr,1,string.find(namestr,"%p")-1);
  namestr = string.sub(namestr,string.find(namestr,"%p")+1);
  name.counter = tonumber(string.sub(namestr,1,string.find(namestr,"%p")-1));
  namestr = string.sub(namestr,string.find(namestr,"%p")+1);
  name.partnum = tonumber(string.sub(namestr,1,string.find(namestr,"%p")-1));
  namestr = string.sub(namestr,string.find(namestr,"%p")+1);
  name.parts = tonumber(namestr);
  return name
end

function push_imu(imu)
  rcm.set_robot_imu(vector.new({imu.acc[1], imu.acc[2], imu.acc[3]}));
  rcm.set_robot_gyro(vector.new({imu.wrpy[1], imu.wrpy[2], imu.wrpy[3]}));
end

function push_lidar(lidar)
  rcm.set_lidar_timestamp(lidar.tstamp);
  rcm.set_lidar_ranges(lidar.range);
  rcm.set_lidar_counter(lidar.counter);
end

function parseImu(str)
  IMU = serialization.deserialize(str);
--  util.ptable(IMU);
--  util.ptable(IMU.arr);
  local imuData = cutil.test_array();
  if IMU.arr then
  --  print(#IMU.arr.data)
    cutil.string2userdata(imuData, IMU.arr.data);
  --  print(type(imuData));
    imu = Serial.imuparser(imuData, IMU.arr.width);
  --  util.ptable(imu)
  --  util.ptable(imu.rpy);
  --  util.ptable(imu.wrpy);
  --  util.ptable(imu.acc);
    imu.tstamp = IMU.timestamp;
    return imu;
  else
    imu = {};
    return imu;
  end
end

function parseLIDAR(str)
  LIDAR = serialization.deserialize(str);
--  util.ptable(LIDAR);
--  util.ptable(LIDAR.arr);
--  print(LIDAR.arr.name)
  lidar = {};
  if LIDAR.arr then
    name = parse_name(LIDAR.arr.name);
  --  util.ptable(name)
    local lidarRange = cutil.test_array();
    cutil.string2userdata(lidarRange, LIDAR.arr.data);
    lidar.tstamp = LIDAR.timestamp;
    lidar.counter = name.counter;
    lidar.range = lidarRange;
  end
  return lidar;
end

--dataPath = os.getenv('HOME')..'/Dropbox/\[Robocup\]\ \(2\)/data/';
dataPath = './data/';
dataStamp = "12.19.2012.21.34";

dataType = 'imu';
imuFile = dataType..dataStamp..'-'..'*'
local imufileList = assert(io.popen('/bin/ls '..dataPath..imuFile, 'r'))
imuFileNum = 0;
for imu in imufileList:lines() do imuFileNum = imuFileNum + 1; end;

dataType = 'lidar';
lidarFile = dataType..dataStamp..'-'..'*'
local lidarfileList = assert(io.popen('/bin/ls '..dataPath..lidarFile, 'r'))
lidarFileNum = 0;
for lidar in lidarfileList:lines() do lidarFileNum = lidarFileNum + 1; end;

--[[
print(imuFileNum, lidarFileNum);
-- traverse imu files
--for cnt = 0, imuFileNum - 1 do
for cnt = 0,0 do
  imuFile = dataPath..'imu'..dataStamp..'-'..cnt;
  print(imuFile);
  imuFile = assert(io.open(imuFile, 'r+'));
  line = imuFile:read();  
  while line ~= nil do
--  for line in imuFile:lines() do
    imu = parseImu(line);
    line = imuFile:read();
  end
--  end
  imuFile:close();
end

-- traverse lidar files
--for cnt = 0, lidarFileNum - 1 do
for cnt = 0, 0 do
  lidarFile = dataPath..'lidar'..dataStamp..'-'..cnt;
  print(lidarFile);
  lidarFile = assert(io.open(lidarFile, 'r+'));
  line = lidarFile:read()
  while line ~= nil do
    lidar = parseLIDAR(line);
    line = lidarFile:read();
  end
  lidarFile:close();
end
--]]
imuFileCounter = 0;
lidarFileCounter = 0;

imuFile = dataPath..'imu'..dataStamp..'-'..imuFileCounter;
imuFile = assert(io.open(imuFile, 'r+'));

lidarFile = dataPath..'lidar'..dataStamp..'-'..lidarFileCounter;
lidarFile = assert(io.open(lidarFile, 'r+'));

-- Open first imu and lidar files
imuline = imuFile:read();
lidarline = lidarFile:read();
flag = true;
tDelay = 0.01 * 1E6;
while flag do
  if imuline == nil then
    imuFile:close();
    imuFileCounter = imuFileCounter + 1;
    imuFile = dataPath..'imu'..dataStamp..'-'..imuFileCounter;
    print(imuFile);
    imuFile = assert(io.open(imuFile, 'r+'));
    imuline = imuFile:read();
    imu = parseImu(imuline);
  else
    imu = parseImu(imuline);
  end
  if lidarline == nil then
    lidarFile:close();
    lidarFileCounter = lidarFileCounter + 1;
    lidarFile = dataPath..'lidar'..dataStamp..'-'..lidarFileCounter;
    print(lidarFile)
    lidarFile = assert(io.open(lidarFile, 'r+'));
    lidarline = lidarFile:read();
    lidar = parseLIDAR(lidarline);
  else
    lidar = parseLIDAR(lidarline);
  end
--  util.ptable(imu);
--  util.ptable(lidar);
  if imu.tstamp ~= nil then
    print('Time stamp: imu',imu.tstamp);
    push_imu(imu);
    imuline = imuFile:read();
  else
    flag = false;
  end
  if lidar.tstamp ~= nil and imu.tstamp ~= nil then
    timediff = math.abs(lidar.tstamp - imu.tstamp);
  --  print(lidar.counter)
  --  print(timediff)
    if timediff < 0.01 then
      print('Time stamp: lidar',lidar.tstamp);
      push_lidar(lidar);
      lidarline = lidarFile:read();
    end
  else
    flag = false;
  end

  unix.usleep(tDelay);
end


imufileList:close();
lidarfileList:close();

