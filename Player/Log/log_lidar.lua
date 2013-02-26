module(... or '', package.seeall)

-- Add the required paths

uname  = io.popen('uname -s')
system = uname:read();

-- Include the proper directories
cwd = '.';
package.cpath = cwd.."/../Lib/?.so;"..package.cpath;
package.path = cwd.."/../Util/?.lua;"..package.path;

local serialization = require('serialization');
require('Hokuyo')
require('signal')
require('Serial');
require('unix');

hokuyo = {}
hokuyo.nReturns = 1081;
--hokuyo.serial = "00805676"
hokuyo.serial = "00907258"
hokuyo.device = "/dev/ttyACM0"
--hokuyo.device = "/dev/tty.usbmodem1411"
Hokuyo.open(hokuyo.device, hokuyo.serial);

function ShutDownFN()
  print("Proper shutdown")
  Hokuyo.shutdown()
  os.exit(1);
end

-- Create files
lidarfilecnt = 0;
function get_filename()
  local filetime = os.date('%m.%d.%Y.%H.%M');
  local filename = 
string.format("/mnt/logs/shadwell/logs/lidar%s-%04d", filetime, 
lidarfilecnt);
  return filename;
end

file = io.open(get_filename(), "w");
linecount = 0;
maxlinecount = 500;

cntr = 0;
cnti = 0;
t0 = unix.time();
t2 = unix.time();
while (1) do
  t1 = unix.time(); -- timestamp

  Hokuyo.update();
  cntr = cntr + 1;
  if (cntr % 40 == 0) then
    print("Scan rate "..40/(unix.time() - t0));
    t0 = unix.time();
  end

  lidar = Hokuyo.retrieve();
  width = hokuyo.nReturns;
  height = 1;
  lidarArray = serialization.serialize_array(lidar.ranges, width,
                height, 'single', 'ranges', lidar.counter);
  savelidar = {};
  savelidar.t = t1;
  savelidar.arr = lidarArray[1];
  local savedata=serialization.serialize(savelidar);
--  print(savedata);
--  savedata = Z.compress(savedata, #savedata);
  file:write(savedata);
  file:write('\n');
  linecount = linecount + 1;
  if linecount > maxlinecount then
    linecount = 0;
    file:close();
    lidarfilecnt = lidarfilecnt + 1;
    file = io.open(get_filename(), "w");
  end

  signal.signal("SIGINT", ShutDownFN);
  signal.signal("SIGTERM", ShutDownFN);

end

file:close();

