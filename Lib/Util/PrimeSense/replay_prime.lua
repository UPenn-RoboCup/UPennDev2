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
package.path = cwd .. '/../World/?.lua;' .. package.path;

require 'unix';
require 'primecm'
-- Load logs
dofile( arg[1] )

run_once = false;
net = false;
t0 = unix.time()
t_init = 0;
n_logs = table.getn(log);

if( net ) then
  Team = require 'TeamPrime'
  Team.entry()
end


for l=1,n_logs do
  local t_start = unix.time();
  local timestamp = log[l].t;
  for i,v in ipairs(primecm.jointNames) do
    --    local pos, rot, confidence = primesense.get_jointtables(i);
    local pos = { log[l].x[i],log[l].y[i],log[l].z[i] };
    local confidence = { log[l].posconf[i],log[l].rotconf[i] };

    primecm['set_position_'..v]( pos );
    --    primecm['set_orientation_'..v]( rot );
    primecm['set_confidence_'..v]( confidence );
    if( net ) then
      Team.update()
    end
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

  -- Debug
  print('Count:',l,'Time:',timestamp );
  -- Timing
  if(l<n_logs) then
    local timestamp_next = log[l+1].t;
    local twait = timestamp_next-timestamp;
    local t_loop = unix.time() - t_start;
    --    print('Count:',l,'Time:',timestamp, 'Wait:',math.max(twait-t_loop,0))    
    unix.usleep( 1e6*math.max(twait-t_loop,0) );
  end

end
-- After done playing, reset the skeleton found variable so no more movement
primecm.set_skeleton_found( 0 );
if( net ) then
  Team.update();
end
