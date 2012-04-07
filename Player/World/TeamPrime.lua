module(..., package.seeall);
require('Comm');
require 'primecm'; -- Sending and receiving Kinect Data
require('gcm');
require 'serialization'

-- Initialization
Comm.init(Config.dev.ip_wired,54321);
teamID   = gcm.get_team_number();
playerID = gcm.get_team_player_id();
msgTimeout = Config.team.msgTimeout;
states = {};

function entry()

end

function recv_msgs()
  while (Comm.size() > 0) do 
    t = serialization.deserialize(Comm.receive());
    if (t and (t.teamNumber) and (t.teamNumber == teamID ) and (t.id) and (t.id ~= playerID)) then
      t.tReceive = Body.get_time();
      states[t.id] = t;
    end
  end
end

function update()
  local state = {};
  -- If we are player 0 then we have a PrimeSense
  if( playerID==0 ) then
    local position = {};
    local confidence = {};
    local orientation = {};
    for i,v in ipairs(primecm.jointNames) do
      position[v] = primecm['get_position_'..v](  );
      --primecm['get_orientation_'..v](  );
      confidence[v] = primecm['get_confidence_'..v](  );
    end
    state.pos = position;
    state.conf = confidence;
    state.found = primecm.get_skeleton_found(  );
    state.t = primecm.get_skeleton_timestamp(  );
  end

  -- Send/Receive Messages depending on if we have a PrimeSense
  if( playerID==0 ) then
    Comm.send(serialization.serialize(state)); 
  else
    recv_msgs();
    local state = states[0];
    if( state ) then
      -- Push to our primecm
      for i,v in ipairs(primecm.jointNames) do
        primecm['set_position_'..v]( state.pos[v] );
        --primecm['get_orientation_'..v](  );
        primecm['get_confidence_'..v]( state.conf[v] );
      end
      primecm.set_skeleton_found( state.found );
      primecm.set_skeleton_timestamp( state.t );
    end
  end

end
