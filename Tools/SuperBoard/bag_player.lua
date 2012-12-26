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
require('cutil');
require('Serial');

function parseImu(str)
  IMU = serialization.deserialize(str);
--  util.ptable(IMU);
--  util.ptable(IMU.arr);
  local imuData = cutil.test_array();
--  print(#IMU.arr.data)
  cutil.string2userdata(imuData, IMU.arr.data);
--  print(type(imuData));
  imu = Serial.imuparser(imuData, IMU.arr.width);
  util.ptable(imu)
--  util.ptable(imu.rpy);
--  util.ptable(imu.wrpy);
--  util.ptable(imu.acc);
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

print(imuFileNum, lidarFileNum);
-- traverse imu files
--for cnt = 0, imuFileNum - 1 do
for cnt = 0,0 do
  imuFile = dataPath..'imu'..dataStamp..'-'..cnt;
  print(imuFile);
  imuFile = assert(io.open(imuFile, 'r+'));
--  line = imuFile:read();  
  for line in imuFile:lines() do
    parseImu(line);
  end
  imuFile:close();
end
--for cnt = 0, lidarFileNum - 1 do
--  lidarFile = 'lidar'..dataStamp..'-'..cnt;
--  print(lidarFile);
--end

--linecounter = 0;
--for file in fileList:lines() do 
--  print(file);
--  local file = io.open(file, 'r+');
--  for line in file:lines() do
--    linecounter = linecounter + 1;
--    print(line)
--  end
--  file:close();
--end
--fileList:close();

imufileList:close();
lidarfileList:close();

