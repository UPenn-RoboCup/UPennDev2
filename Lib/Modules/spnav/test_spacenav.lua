module(... or '', package.seeall)

-- Add the required paths
cwd = '.';

uname  = io.popen('uname -s')
system = uname:read();

package.cpath = cwd.."/?.so;"..package.cpath;
package.cpath = cwd.."/../../Player/Lib/?.so;"..package.cpath;
package.path = cwd.."/../../Player/Util/?.lua;"..package.path;
package.path = cwd.."/../../Player/Config/?.lua;"..package.path;

require('signal')
require('unix')
local util = require 'util'

require('Spacenav')

function ShutDownFN()
  print("Proper shutdown")
  Spacenav.shutdown()
  os.exit(1);
end

Spacenav.open()

cntr = 0;
t0 = unix.time();
while (true) do
  cntr = cntr + 1;
  if (cntr % 40 == 0) then
--    print("Scan rate "..40/(unix.time() - t0));
    t0 = unix.time();
  end

  tbl = Spacenav.get()
  if type(tbl) == 'table' then
    if tbl.event == 'motion' then
      print(tbl.x, tbl.y, tbl.z, tbl.rx, tbl.ry, tbl.rz)
  --  end
    elseif tbl.event == 'button' then
      print(tbl.bnum, tbl.bpress)
    end
  end

  unix.usleep(330)

  signal.signal("SIGINT", ShutDownFN);
  signal.signal("SIGTERM", ShutDownFN);
end
