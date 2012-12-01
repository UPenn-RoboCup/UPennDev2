module(... or '', package.seeall)

-- Add the required paths
cwd = '.';

uname  = io.popen('uname -s')
system = uname:read();

package.cpath = cwd.."/?.so;"..package.cpath;
package.cpath = cwd.."/../../../Player/Lib/?.so;"..package.cpath;
package.path = cwd.."/../../../Player/Util/?.lua;"..package.path;
package.path = cwd.."/../../../Player/Config/?.lua;"..package.path;

require('Hokuyo')
require('signal')
require('unix')
require('hcm')

function ShutDownFN()
  print("Proper shutdown")
  Hokuyo.shutdown()
  os.exit(1);
end

hokuyo = {}
hokuyo.serial = "00805676"
--hokuyo.serial = "00907258"
hokuyo.device = "/dev/ttyACM0"
Hokuyo.open(hokuyo.device, hokuyo.serial);

cntr = 0;
t0 = unix.time();
while (true) do
  Hokuyo.update();
  cntr = cntr + 1;
  if (cntr % 40 == 0) then
    print("Scan rate "..40/(unix.time() - t0));
    t0 = unix.time();
  end

  lidar = Hokuyo.retrieve();
  
  hcm.set_hokuyo_counter(lidar.counter);
  hcm.set_hokuyo_id(lidar.id);
  hcm.set_hokuyo_ranges(lidar.ranges);
  hcm.set_hokuyo_startAngle(lidar.startAngle);
  hcm.set_hokuyo_stopAngle(lidar.stopAngle);
  hcm.set_hokuyo_startTime(lidar.startTime);
  hcm.set_hokuyo_startTime(lidar.startTime);
  

  signal.signal("SIGINT", ShutDownFN);
  signal.signal("SIGTERM", ShutDownFN);
end
