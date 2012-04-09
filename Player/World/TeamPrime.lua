module(..., package.seeall);
require('Comm');
require 'primecm'; -- Sending and receiving Kinect Data
require('gcm');
require 'serialization'

-- Initialization
print("My address:",Config.dev.ip_wired)
Comm.init(Config.dev.ip_wired,54321);
teamID   = gcm.get_team_number();
playerID = gcm.get_team_player_id();
msgTimeout = Config.team.msgTimeout;
states = {};
state = {};

function entry()

end

function recv_msgs()
  while (Comm.size() > 0) do 
    t = serialization.deserialize(Comm.receive());
    if (t and (t.tid) and (t.tid == teamID ) and (t.id) and (t.id ~= playerID)) then
      t.tReceive = Body.get_time();
      states[t.id] = t;
    end
  end
end

function update()
  state.id = playerID;
  state.tid = teamID;
  -- If we are player 0 then we have a PrimeSense
  if( playerID==0 ) then
    state.f = primecm.get_skeleton_found(  );
    state.t = primecm.get_skeleton_timestamp(  );
    local position = {};
    local confidence = {};
    local orientation = {};
    for i,v in ipairs(primecm.jointNames) do
      position[i] = primecm['get_position_'..v](  );
      --orientation[i] = primecm['get_orientation_'..v](  );
      confidence[i] = primecm['get_confidence_'..v](  );
    end
    state.o = orientation;    
    state.p = position;
    state.c = confidence;
  end

  -- Send/Receive Messages depending on if we have a PrimeSense
--  print('PlayerId',playerID,'teamID',teamID)
  if( playerID==0 ) then
--    print('Sending',serialization.serialize(state) )
    local ret = Comm.send(serialization.serialize(state)); 
    --print(ret)
  else
    recv_msgs();
    local state = states[0];
    if( state ) then
      -- Push to our primecm
      for i,v in ipairs(primecm.jointNames) do
        primecm['set_position_'..v]( state.p[i] );
        --primecm['get_orientation_'..v](  );
        primecm['get_confidence_'..v]( state.c[i] );
      end
      primecm.set_skeleton_found( state.f );
      primecm.set_skeleton_timestamp( state.t );
    end
  end

end
