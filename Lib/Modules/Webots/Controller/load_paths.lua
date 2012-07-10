module(... or '', package.seeall)

-- Get Platform for package path
uname  = io.popen('uname -s')
system = uname:read();
pwd = io.popen('pwd');
cwd = pwd:read();

print('I am on '..system..' in '..cwd)

webots = false;
local platform = os.getenv('PLATFORM') or '';

if (string.find(platform,'webots')) then
  cwd = cwd .. '/Player';
  webots = true;
end


if (string.find(cwd,'Player/Lib')) then
  cwd = cwd .. '/..';
  print('Using library directory...')
end

if( webots ) then
  print('Using webots!')
end
-- Get Computer for Lib suffix
local computer = os.getenv('COMPUTER') or system;
  package.cpath = cwd .. '/Lib/?.so;' .. package.cpath;

package.path = cwd .. '/?.lua;' .. package.path;
package.path = cwd .. '/Util/?.lua;' .. package.path;
package.path = cwd .. '/Config/?.lua;' .. package.path;
package.path = cwd .. '/Lib/?.lua;' .. package.path;
package.path = cwd .. '/Dev/?.lua;' .. package.path;
package.path = cwd .. '/Motion/?.lua;' .. package.path;
package.path = cwd .. '/Motion/Walk/?.lua;' .. package.path;
package.path = cwd .. '/Motion/keyframes/?.lua;' .. package.path;
package.path = cwd .. '/Vision/?.lua;' .. package.path;
package.path = cwd .. '/World/?.lua;' .. package.path;
require 'unix'

if( not webots ) then
  require('getch')
  getch.enableblock(1);
  unix.usleep(1.0*1E5);
end

function get_key_byte()
  if(webots) then
    byte = controller.wb_robot_keyboard_get_key();
    -- Webots only return captal letter number
    if byte>=65 and byte<=90 then
      byte = byte + 32;
    end
  else
    byte = string.byte(getch.get(),1);
  end
  return byte;
end

print('Loaded default paths!')
io:flush();

