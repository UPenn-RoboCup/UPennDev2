module(... or "", package.seeall)

-- Get Platform for package path
cwd = os.getenv('PWD');
local platform = os.getenv('PLATFORM') or '';
if (string.find(platform,'webots')) then cwd = cwd .. '/Player';
end

-- Get Computer for Lib suffix
local computer = os.getenv('COMPUTER') or '';
computer = 'Darwin'
if (string.find(computer, 'Darwin')) then
  -- MacOS X uses .dylib:
  package.cpath = cwd .. '/?.dylib;' .. package.cpath;
else
  package.cpath = cwd .. '/?.so;' .. package.cpath;
end

package.path = cwd .. '/../Util/?.lua;' .. package.path;
package.path = cwd .. '/../Config/?.lua;' .. package.path;

require 'primesense'
require 'unix';
require 'primecm'

-- Wait at least 1 seconds
--unix.sleep(10);
t0 = unix.time()
while(true) do
  local torso = primesense.get_torso()
  
  if( torso ) then
    print( "Torso: ", unpack(torso) );
    print();
    -- Update Shm
    primecm.set_skeleton_torso( torso );
    primecm.set_skeleton_found( 1 );
  else
    primecm.set_skeleton_found( 0 );    
    print("No user detected... Waiting 1 second...")
    unix.sleep(1);
  end
  --Pause .1 seconds
  unix.usleep(1E5);
end
