cwd = os.getenv('PWD')
require('init')

require 'gcm'
teamID   = gcm.get_team_number();
playerID = gcm.get_team_player_id();
nPlayers = gcm.get_game_nplayers();

-- Always require self
require('primecm'..playerID)

--for i=1,nPlayers do
for i=1,2 do
  if( i~= playerID ) then -- redundant
    require('primecm'..i)
  end
end


-- Broadcast over the network
--net = true;
net = false;
-- run from log file (this is set automatically)
-- lua run_prime XXX.lua where XXX.lua is the log
logs = false;
-- box runs punch detection code
box = false;
if( arg[1] ) then
  dofile( arg[1] )
  logs = true;
else
  require 'primesense'  
end

--if( box ) then
require 'libboxer'
--end

-- Issue debug line telling which mode we are in
desired_fps = 20;
print '\n\n=====================';
print('Desired FPS: ',desired_fps);
print('Run Network?',net);
print('Run Logs?',logs);
print('Team/Player',teamID..'/'..playerID);
print('nPlayers',nPlayers);
print '=====================';

if( not logs ) then
  unix.usleep(1e6*2);
end
t0 = unix.time();
timestamp0 = t0;
t_init = 0;
if( logs ) then
  n_logs = table.getn(log);
  local it = 1;
  timestamp0 = log[it].t;
  it = it+1;
  while(not timestamp0) do
    timestamp0 = log[it].t;
    it = it+1;
  end
end

print(Config.dev.team..' entry...')
io:flush()

if( net ) then
  require 'Team'
  Team.entry()
end

print('Entering Loop...')
io:flush()

count = 0;
init = false;
primecm.set_skeleton_found( {0,0} );
while( not logs or count<n_logs ) do
  count = count + 1;
  if( not logs ) then
    ret = primesense.update_joints();
    timestamp = unix.time();
  else
    if( timestamp and log[count].t<timestamp ) then
      timestamp0 = log[count].t;
    end
    timestamp = log[count].t;
  end
  t_start = unix.time();
  if( logs or ret ) then
    if( not init and not logs ) then
      init = true;
      timestamp0 = timestamp;
    end
    -- Check each player
    -- Is each player active?
    active = {0,0}; -- init as no...
    for i,v in ipairs(primecm.jointNames) do
      if( logs ) then
        pos = { log[count].x[i],log[count].y[i],log[count].z[i] };
        confidence = { log[count].posconf[i],log[count].rotconf[i] };
        myactive = 1;
        myactive2 = 1;
      else
        pos, rot, confidence, myactive = primesense.get_jointtables(1,i);
        pos2, rot2, confidence2, myactive2 = primesense.get_jointtables(2,i);
      end
      active[1] = myactive;
      active[2] = myactive2;
      if( not logs ) then
        pos = vector.new(pos)/1000;
        pos2 = vector.new(pos2)/1000;
      end

      -- Update the SHM for each player, only if it is active
      if( active[1]>0 ) then
        primecm['set_position_'..v]( pos );
        if( not logs ) then
          primecm['set_orientation_'..v]( rot );
        end
        primecm['set_confidence_'..v]( confidence );
      end


    end

    -- Update the global primesense information
    primecm.set_skeleton_found( active );
    primecm.set_skeleton_timestamp( timestamp-timestamp0 );
  end

  if( box ) then
    libboxer.update(1);
    libboxer.update(2);
    local punch = primecm.get_joints_punch();
    if( punch[1]>0 ) then
      print( "Punch1: ", punch[1])
    elseif( punch[2]>0 ) then
      print( "Punch2: ", punch[2])
    end
  end

  -- Update the Team information
  if( net ) then
    Team.update()
  end

  --[[
  -- Done running
  local byte = get_key_byte();
  if( byte==string.byte('e') ) then
  primecm.set_skeleton_enabled( 1-primecm.get_skeleton_enabled() );
  end
  if(byte==string.byte('q')) then
  break;
  end
  --]]

  libboxer.get_torso_orientation();


  -- Timing
  if( init and not logs ) then
    if( count % 30==0 ) then
      local fps = 30 / (unix.time()-(t_count or 0))
      t_count = unix.time();
      print('FPS: ',fps)
      print('Time',timestamp-timestamp0 );
      print('User 1: ', active[1]);
      print('User 2: ', active[2]);
      print();
    end
    local t_loop = unix.time() - t_start;
    local twait = 1/desired_fps;
    --    unix.usleep( 1e6*math.max(twait-t_loop,0) );
  end
  if(logs and count<n_logs) then
    print('Count:',count,'Time:',timestamp );    
    local timestamp_next = log[count+1].t;
    local twait = timestamp_next-timestamp;
    local t_loop = unix.time() - t_start;
    unix.usleep( 1e6*math.max(twait-t_loop,0) );
  end

end
-- After done playing, reset the skeleton found variable so no more movement
print('Finished!')
primecm.set_skeleton_found( {0,0} );
if( net ) then
  Team.update();
end

