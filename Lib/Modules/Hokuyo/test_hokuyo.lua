module(... or '', package.seeall)

-- Add the required paths
cwd = '.';

uname  = io.popen('uname -s')
system = uname:read();

package.cpath = cwd.."/?.so;"..package.cpath;
package.cpath = cwd.."/../../../Player/Lib/?.so;"..package.cpath;

require('Hokuyo')
require('signal')
require('unix')

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

  signal.signal("SIGINT", ShutDownFN);
  signal.signal("SIGTERM", ShutDownFN);
end
