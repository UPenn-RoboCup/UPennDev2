module(..., package.seeall);

require('Config');
require('Body');
require('Comm');
require('Speak');
require('vector');
require('serialization');

require('wcm');
require('gcm');

Comm.init(Config.dev.ip_wireless,54321);
print('Receiving Team Message From',Config.dev.ip_wireless);
playerID = gcm.get_team_player_id();

msgTimeout = Config.team.msgTimeout;
nonAttackerPenalty = Config.team.nonAttackerPenalty;
nonDefenderPenalty = Config.team.nonDefenderPenalty;

--Player ID: 1 to 5
--Role: 0 goalie / 1 attacker / 2 defender / 3 supporter 
--4 reserve player / 5 reserve goalie

count = 0;

state = {};
state.teamNumber = gcm.get_team_number();
state.id = playerID;
state.teamColor = gcm.get_team_color();
state.time = Body.get_time();
state.role = -1;
state.pose = {x=0, y=0, a=0};
state.ball = {t=0, x=1, y=0};
state.attackBearing = 0.0;--Why do we need this?
state.penalty = 0;
state.tReceive = Body.get_time();
state.battery_level = wcm.get_robot_battery_level();
state.fall=0;

states = {};
states[playerID] = state;

function pack_msg(state)
  --Tightly pack the state info into a short string

  ------------------------------
  --ID        
  --TeamNo
  --TeamColor 
  --role
  --penalty
  --fall
 
  --ballx bally ballt
  --posex posey posea
  --time
  --battery
  msg_str=string.format(
	"{%d,%d,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.1f",
	state.id, state.teamNumber, state.teamColor,
	state.role,state.penalty,state.fall,
	state.ball.x,state.ball.y,state.ball.t,
	state.pose.x,state.pose.y,state.pose.a,
	state.time,state.battery_level);
  return msg_str;
end

function unpack_msg(msg)
  local state={};
  state.id=msg[1] or 0;
  state.teamNumber=msg[2] or 0;
  state.teamColor=msg[3] or 0;
  state.role=msg[4];
  state.penalty=msg[5];
  state.fall=msg[6];
  state.ball={};
  state.ball.x=msg[7];
  state.ball.x=msg[8];
  state.ball.x=msg[9];
  state.pose={};
  state.pose.x=msg[10];
  state.pose.x=msg[11];
  state.pose.x=msg[12];
  state.pose.time=msg[13];
  state.pose.battery_level=msg[14];
  return state;
end

function recv_msgs()
  while (Comm.size() > 0) do 
--    t = serialization.deserialize(Comm.receive());
    t = unpack_msg(Comm.receive());
    if (t and (t.teamNumber) and (t.teamNumber == state.teamNumber) and (t.id) and (t.id ~= playerID)) then
      t.tReceive = Body.get_time();
      states[t.id] = t;
    end
  end
end

function entry()
end

function update()
  count = count + 1;

  state.time = Body.get_time();
  state.teamNumber = gcm.get_team_number();
  state.teamColor = gcm.get_team_color();
  state.pose = wcm.get_pose();
  state.ball = wcm.get_ball();
  state.role = role;
  state.attackBearing = wcm.get_attack_bearing();
  state.battery_level = wcm.get_robot_battery_level();
  state.fall=wcm.get_robot_is_fall_down();

  if gcm.in_penalty() then
    state.penalty = 1;
  else
    state.penalty = 0;
  end

  if (math.mod(count, 1) == 0) then

--    msg=serialization.serialize(state);

-- Unpacked msg size: 367, packed msg size: 48
-- We can send more vision info wireless as wel....
--    print("Team message size:",string.len(msg))
--    msg2=pack_msg(state);
--    print("Packed team message size:",string.len(msg2))
--    Comm.send(serialization.serialize(state));

    msg=pack_msg(state);
    Comm.send(msg);

    --Copy of message sent out to other players
    state.tReceive = Body.get_time();
    states[playerID] = state;
  end

  -- receive new messages
  recv_msgs();

  -- eta and defend distance calculation:
  eta = {};
  ddefend = {};
  roles = {};
  t = Body.get_time();
  for id = 1,5 do 

    if not states[id] then
      -- no message from player have been received
      eta[id] = math.huge;
      ddefend[id] = math.huge;
      roles[id]=4;
    else
      -- eta to ball
      -- TODO: consider angle as well
      rBall = math.sqrt(states[id].ball.x^2 + states[id].ball.y^2);
      tBall = states[id].time - states[id].ball.t;
      eta[id] = rBall/0.10 + 4*math.max(tBall-1.0,0);
      roles[id]=states[id].role;
      
      -- distance to goal
      dgoalPosition = vector.new(wcm.get_goal_defend());
      pose = wcm.get_pose();
      ddefend[id] = math.sqrt((pose.x - dgoalPosition[1])^2 + (pose.y - dgoalPosition[2])^2);

      if (states[id].role ~= 1) then       -- Non attacker penalty:
        eta[id] = eta[id] + nonAttackerPenalty;
      end

      if (states[id].role ~= 2) then        -- Non defender penalty:
        ddefend[id] = ddefend[id] + 0.3;
      end

      --Ignore goalie, reserver, penalized player
      if (states[id].penalty > 0) or 
	(t - states[id].tReceive > msgTimeout) or
	(states[id].role >3) or 
	(states[id].role ==0) then
        eta[id] = math.huge;
        ddefend[id] = math.huge;
      end
    end
  end

--[[
  if count % 100 == 0 then
    print('---------------');
    print('eta:');
    util.ptable(eta)
    print('ddefend:');
    util.ptable(ddefend)
    print('---------------');
  end
--]]

  -- goalie and reserve player never changes role
  if role~=0 and role<4 then 
    minETA, minEtaID = min(eta);
    if minEtaID == playerID then set_role(1);--attack
    else
      -- furthest player back is defender
      minDDefID = 0;
      minDDef = math.huge;
      for id = 1,5 do
        if id ~= minEtaID and 	   
	ddefend[id] <= minDDef and
	roles[id]<4 then --goalie and reserve don't count
          minDDefID = id;
          minDDef = ddefend[id];
        end
      end
      if minDDefID == playerID then
        set_role(2);    -- defense 
      else
        set_role(3);    -- support
      end
    end
  end

  -- update shm
  update_shm() 
end

function update_shm() 
  -- update the shm values
  gcm.set_team_role(role);
end

function exit()
end

function get_role()
  return role;
end

function set_role(r)
  if role ~= r then 
    role = r;
    Body.set_indicator_role(role);
    if role == 1 then
      -- attack
      Speak.talk('Attack');
    elseif role == 2 then     -- defend
      Speak.talk('Defend');
    elseif role == 3 then     -- support
      Speak.talk('Support');
    elseif role == 0 then     -- goalie
      Speak.talk('Goalie');
    elseif role == 4 then     -- reserve player
      Speak.talk('Reserve Player');
    elseif role == 5 then     -- reserve goalie
      Speak.talk('Reserve Goalie');
    else
      -- no role
      Speak.talk('ERROR: Unknown Role');
    end
  end
  update_shm();
end

--NSL role can be set arbitarily, so use config value
set_role(Config.game.role or 1);

function get_player_id()
  return playerID; 
end

function min(t)
  local imin = 0;
  local tmin = math.huge;
  for i = 1,#t do
    if (t[i] < tmin) then
      tmin = t[i];
      imin = i;
    end
  end
  return tmin, imin;
end
