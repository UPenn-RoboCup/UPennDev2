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
run_once = false;

init = false; -- Not initialized yet
t0 = unix.time()
t_init = 0;
while(true) do
  -- Head is first joint
  local ret = primesense.update_joints()
  local timestamp = unix.time();
  if( ret ) then
    if( not init ) then
      -- Skip first Joint reading
      print('Initialized for Lua!')
      init = true;
      t_init = timestamp
    else
      for i,v in ipairs(primecm.jointNames) do
        local pos, rot, confidence = primesense.get_jointtables(i);
        primecm['set_position_'..v]( pos );
        primecm['set_orientation_'..v]( rot );
        primecm['set_confidence_'..v]( confidence );
        if(run_once) then
          print( v, unpack(pos) );
        end
      end
      -- Update Shm
      primecm.set_skeleton_found( 1 );
      primecm.set_skeleton_timestamp( timestamp );

      if(run_once) then
        return;
      end

    end
  else
    init = false;
    primecm.set_skeleton_found( 0 );    
  end

end

