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

Hokuyo.open();

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
  for k, v in pairs(lidar) do
    print(k,v)
  end

  signal.signal("SIGINT", ShutDownFN);
  signal.signal("SIGTERM", ShutDownFN);
end
