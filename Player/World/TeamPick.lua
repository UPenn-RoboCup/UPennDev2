module(..., package.seeall);
require('Comm');

require('gcm');
require 'vcm'
require 'Z'
require 'serialization'
require 'unix'
require 'util'
require 'pickercm'

wired = true;
ps = false;

if (string.find(Config.platform.name,'Webots')) then
  print('TeamPick: On webots!')
  webots = true;
else
  print('TeamPick: Real robot!')
end

-- Initialization
if( wired ) then
  print("My address:",Config.dev.ip_wired)
  Comm.init(Config.dev.ip_wired,Config.dev.ip_wired_port)
else
  print("My address:",Config.dev.ip_wireless)
  Comm.init(Config.dev.ip_wireless,Config.dev.ip_wireless_port);
end

teamID   = gcm.get_team_number();
playerID = gcm.get_team_player_id();
nPlayers = gcm.get_game_nplayers();

msgTimeout = Config.team.msgTimeout or 2;
compressPick = true;
states = {};
state = {};

function entry( prime )
  if(prime) then
    print('Using the PrimeSense for control!')  
    ps = true;
  end
end

function recv_msgs()
  while (Comm.size() > 0) do
    msg = Comm.receive()
    if compressPick then
      msg = Z.uncompress(msg, #msg)
    end
    t = serialization.deserialize(msg);
    if t and t.k then
      t.tReceive = unix.time();
      states[0] = t;
    end
  end
end

function update()
  if( webots ) then
    return;
  end

  if( ps ) then -- We have a primesense
    send_body();
  else
    recv_msgs();

    -- Check when we last updated
    if( not states[0] or (unix.time() - states[0].tReceive > msgTimeout) ) then
      pickercm.set_device_kind( 0 );
      return;
    end

    local kind = states[0].k;
    pickercm.set_device_kind(kind)
    if kind == 1 then --kinect
      -- Set the walking engine
      pickercm.set_desired_velocity( states[0].vel );
      pickercm.set_desired_pLArm(states[0].pL)
      pickercm.set_desired_pRArm(states[0].pR)
      pickercm.set_desired_rpy( states[0].rpy );
      pickercm.set_desired_bodyHeight(states[0].bH)
    elseif kind==2 then --xbox controller
      pickercm.set_desired_velocity( states[0].vel );
      pickercm.set_desired_dpLArm(states[0].pL)
      pickercm.set_desired_dpRArm(states[0].pR)
      pickercm.set_desired_rpy( states[0].rpy );
      pickercm.set_desired_dbodyHeight(states[0].bH)
    end

    --[[
    local ph_footX = (Config.walk.bodyHeight-states[0].bH) 
    / (Config.walk.bodyHeight-Config.stance.bodyHeightSit)
    local footX = ph_footX*-0.05 + (1-ph_footX)*mcm.get_footX();
    pickercm.set_walk_footX(footX);
    --]]
  end

end

function send_body()

  -- Organize the data
  state = {};
  state.k  = pickercm.get_device_kind();

  if state.k == 1 then --kinect
    state.vel = pickercm.get_desired_velocity();
    state.bH  = pickercm.get_desired_bodyHeight();
    state.pL  = pickercm.get_desired_pLArm();
    state.pR  = pickercm.get_desired_pRArm();
    state.rpy = pickercm.get_desired_rpy();
  elseif state.k==2 then --xbox
    state.vel = pickercm.get_desired_velocity();
    state.bH  = pickercm.get_desired_dbodyHeight();
    state.pL  = pickercm.get_desired_dpLArm();
    state.pR  = pickercm.get_desired_dpRArm();
    state.rpy = pickercm.get_desired_rpy();
  end
  -- Burst mode
  local ser = serialization.serialize(state)
  if compressPick then
    ser = Z.compress(ser, #ser)
  end
  local ret = Comm.send( ser, #ser );
  ret = Comm.send( ser, #ser );
  ret = Comm.send( ser, #ser );
  ret = Comm.send( ser, #ser );

  print('Sent: '..ret..' bytes')
  --  print('Sent: '..ret..' bytes',ser)
end

function exit()
end
