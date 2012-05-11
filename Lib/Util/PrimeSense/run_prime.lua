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
net = true;
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
print('Team/Player',teamID..'/'..playerID);
print '=====================';
unix.usleep(1e6*2);

t0 = unix.time();
timestamp0 = t0;
t_init = 0;
if( logs ) then
  n_logs = table.getn(log);
  timestamp0 = log[1].t;
end
if( net ) then
  Team = require 'TeamPrime'
  Team.entry()
end

count = 0;
init = false;
desired_fps = 20;
primecm.set_skeleton_found( 0 );
while( not logs or count<n_logs ) do
  count = count + 1;
  if( not logs ) then
    ret = primesense.update_joints();
    timestamp = unix.time();
  else
    timestamp = log[count].t;
  end
  t_start = unix.time();
  if( logs or ret ) then
    if( not init and not logs ) then
      init = true;
      timestamp0 = timestamp;
    end
    for i,v in ipairs(primecm.jointNames) do
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
    primecm.set_skeleton_timestamp( timestamp-timestamp0 );
  end

  -- Done running
  if(run_once) then
    return;
  end

  -- Timing
  if( init and not logs ) then
    print('Time',timestamp-timestamp0 );
    local t_loop = unix.time() - t_start;
    local twait = 1/desired_fps;
--    unix.usleep( 1e6*math.max(twait-t_loop,0) );
  end
  if(logs and count<n_logs) then
    print('Count:',count,'Time:',timestamp );    
    local timestamp_next = log[count+1].t;
    local twait = timestamp_next-timestamp;
    local t_loop = unix.time() - t_start;
  end

end
-- After done playing, reset the skeleton found variable so no more movement
primecm.set_skeleton_found( 0 );
if( net ) then
  Team.update();
end

