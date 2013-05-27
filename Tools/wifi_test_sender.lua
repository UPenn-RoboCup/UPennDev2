local cwd = os.getenv('PWD')
package.cpath = cwd..'/../Lib/Modules/Util/Unix/?.so;'..package.cpath
package.cpath = cwd..'/../Lib/Modules/Util/Shm/?.so;'..package.cpath
package.cpath = cwd..'/../Lib/Modules/Util/CArray/?.so;'..package.cpath
package.cpath = cwd..'/../Lib/Modules/Comm/?.so;'..package.cpath
package.path = cwd..'/../Player/Config/?.lua;'..package.path
package.path = cwd..'/../Player/Util/?.lua;'..package.path

require 'util'
require 'Config'
require 'unix'
require 'Comm'
require 'util'

Comm.init(Config.dev.ip_wireless, Config.dev.ip_wireless_port);

while (true) do
  t0 = unix.time();
  -- print('send timestamp', t0)
  t0_str = tostring(t0);
  Comm.send(t0_str, #t0_str);

  msg = Comm.receive()
  while (msg == nil)  do
    msg = Comm.receive()
  end
  t1 = unix.time()
  print('Latency', 'send:'..t0, 'recv:'..msg,  unix.time() - tonumber(msg))

  unix.usleep(1E6 * 0.005)
end
