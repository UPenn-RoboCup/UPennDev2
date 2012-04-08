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
require 'gcm'
run_once = false;
net = false;
logs = false;
if( arg[1] ) then
  dofile( arg[1] )
  logs = true;
else
  require 'primesense'  
end

-- Issue debug line telling which mode we are in
teamID   = gcm.get_team_number();
playerID = gcm.get_team_player_id();
print '\n\n=====================';
print('Run Once?',run_once);
print('Run Network?',net);
print('Run Logs?',logs);
print('Team/Player'..teamID..'/'..playerID);
print '=====================';
unix.usleep(1e6*2)

t0 = unix.time()
t_init = 0;
if( logs ) then
  n_logs = table.getn(log);
end
if( net ) then
  Team = require 'TeamPrime'
  Team.entry()
end

count = 0;
while( not logs or count<=n_logs ) do
  count = count + 1;
  if( not logs ) then
    ret = primesense.update_joints();
    timestamp = unix.time();
  else
    timestamp = log[count].t;
    t_start = unix.time();    
  end
  if( logs or ret ) then
    for i,v in ipairs(primecm.jointNames) do
      -- local pos, rot, confidence = primesense.get_jointtables(i);
      if( logs ) then
        pos = { log[count].x[i],log[count].y[i],log[count].z[i] };
        confidence = { log[count].posconf[i],log[count].rotconf[i] };
      else
        pos, rot, confidence = primesense.get_jointtables(i);
      end

      primecm['set_position_'..v]( pos );
      if( not logs ) then
        primecm['set_orientation_'..v]( rot );
      end
      primecm['set_confidence_'..v]( confidence );
      if( net ) then
        Team.update()
      end
      -- Debug all the joints
      if(run_once) then
        print( v, unpack(pos) );
      end
    end
    primecm.set_skeleton_found( 1 );
    primecm.set_skeleton_timestamp( timestamp );
  else -- Not running from logs, and primesense does not detect anyone
    primecm.set_skeleton_found( 0 );
  end

  -- Done running
  if(run_once) then
    return;
  end

  -- Timing
  if(logs and count<n_logs) then
    print('Count:',count,'Time:',timestamp );    
    local timestamp_next = log[count+1].t;
    local twait = timestamp_next-timestamp;
    local t_loop = unix.time() - t_start;
    unix.usleep( 1e6*math.max(twait-t_loop,0) );
  end

end
-- After done playing, reset the skeleton found variable so no more movement
primecm.set_skeleton_found( 0 );
if( net ) then
  Team.update();
end

