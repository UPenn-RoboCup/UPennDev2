module(..., package.seeall);

require('Config');
require('Body');
require('Comm')
require('Speak');
require('vector');
require('serialization');
require('wcm');
require('gcm');


playerID = gcm.get_team_player_id();
msgTimeout = Config.team.msgTimeout;
nonAttackerPenalty = Config.team.nonAttackerPenalty;
nonDefenderPenalty = Config.team.nonDefenderPenalty;

role = 1;

count = 0;

state = {};
state.teamNumber = gcm.get_team_number();
state.id = playerID;
state.teamColor = gcm.get_team_color();
state.time = Body.get_time();
state.role = role;
state.pose = {x=0, y=0, a=0};
state.ball = {t=0, x=1, y=0};
state.attackBearing = 0.0;
state.penalty = 0;
state.tReceive = Body.get_time();
state.battery_level = wcm.get_robot_battery_level();

states = {};
states[playerID] = state;

function recv_msgs()

--We need to update object positions via GPS

  while (Comm.size() > 0) do 
    msg=Comm.receive();
    --Ball GPS Info hadling
    if msg and #msg==20 then --Ball position message
      obj_gpsx=(tonumber(string.sub(msg,2,6))-5)*2;
      obj_gpsy=(tonumber(string.sub(msg,8,12))-5)*2;
      obj_gpsz=(tonumber(string.sub(msg,14,18))-5)*2;
      wcm.set_robot_gps_ball({obj_gpsx,obj_gpsy,obj_gpsz});
--    print("Object pos:",obj_gpsx,obj_gpsy,obj_gpsz)
    end
  end
end

function entry()
end

function update()
  --Don't send or receive messages
  --Don't change roles

  count = count + 1;

  recv_msgs();
  set_role(1); --Always attacker

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
    elseif role == 2 then
      -- defend
      Speak.talk('Defend');
    elseif role == 3 then
      -- support
      Speak.talk('Support');
    elseif role == 0 then
      -- goalie
      Speak.talk('Goalie');
    else
      -- no role
      Speak.talk('ERROR: Unknown Role');
    end
  end
end
-- Webots has id=0 map to goalie.  Real robots has id=1 map to goalie
if (string.find(Config.platform.name,'Webots')) then
  set_role(playerID);
else
  set_role(playerID-1);
end

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
