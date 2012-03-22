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
  -- Head is first joint
  local ret = primesense.update_joints()

  if( ret ) then
    for i,v in ipairs(primecm.jointNames) do
      local rot, confidence = primesense.get_jointtables(i);
      print( #rot );
      print( #confidence );
      
      --primecm['set_position_'..v]( pos );
      primecm['set_orientation_'..v]( rot );
      primecm['set_confidence_'..v]( confidence );
    end
    print();
    -- Update Shm
    primecm.set_skeleton_found( 1 );
  else
    primecm.set_skeleton_found( 0 );    
  end

  -- Run at 10Hz
  unix.usleep(1E5);
end
