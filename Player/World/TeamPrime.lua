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
  -- If we are player 0 then we have a PrimeSense
  if( playerID==0 ) then
    local found = primecm.get_skeleton_found(  );
    local timestamp = primecm.get_skeleton_timestamp(  );
    local position = {};
    local confidence = {};
    local orientation = {};
    for i,v in ipairs(primecm.jointNames) do
      position[i] = primecm['get_position_'..v](  );
      orientation[i] = primecm['get_orientation_'..v](  );
      confidence[i] = primecm['get_confidence_'..v](  );
    end
    --[[
    local stateo = {};
    stateo.t = timestamp;
    stateo.tid = teamID;
    stateo.id = playerID;
    stateo.f = found;
    stateo.o = orientation;
    local reto = Comm.send(serialization.serialize(stateo));
    --]]
    local reto = 0;
    local statep = {};
    statep.t = timestamp;
    statep.tid = teamID;
    statep.id = playerID;
    statep.f = found;
    statep.p = position;
    local retp = Comm.send(serialization.serialize(statep));
    local statec = {};
    statec.t = timestamp;
    statec.tid = teamID;
    statec.id = playerID;
    statec.f = found;
    statec.c = confidence;    
    local retc = Comm.send(serialization.serialize(statec));
    if( reto==-1 or retp==-1 or retc==-1 ) then
      -- Problem!
      print('TeamPrime Problem!',reto,retp,retc);
    end
  else
    recv_msgs();
    local state = states[0];
    if( state ) then
      -- Push to our primecm
      for i,v in ipairs(primecm.jointNames) do
        if( state.p ) then
          primecm['set_position_'..v]( state.p[i] );
        end
        if( state.o ) then
          primecm['get_orientation_'..v](  );
        end
        if( state.c ) then
          primecm['get_confidence_'..v]( state.c[i] );
        end
      end
      primecm.set_skeleton_found( state.f );
      primecm.set_skeleton_timestamp( state.t );
    end
  end

end
