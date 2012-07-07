module(..., package.seeall);

require('vector')
require 'unix'
local robotName=unix.gethostname();

platform = {}; 
platform.name = 'xos'

function loadconfig(configName)
  local localConfig=require(configName);
  for k,v in pairs(localConfig) do
    Config[k]=localConfig[k];
  end
end

--Robot CFG should be loaded first to set PID values
loadconfig('Robot/Config_XOS_Robot') 
loadconfig('Walk/Config_XOS_Walk')
loadconfig('World/Config_XOS_World')
loadconfig('Kick/Config_XOS_Kick')
loadconfig('Vision/Config_XOS_Vision')
--Location Specific Camera Parameters--
loadconfig('Vision/Config_XOS_Camera_Mexico_LC')

-- Device Interface Libraries
dev = {};
dev.body = 'XOSBody'; 
dev.camera = 'XOSCam';
dev.kinematics = 'XOSKinematics';
dev.ip_wired = '192.168.123.255';
dev.ip_wireless = '192.168.126.255';
dev.ip_wireless_port = 54321;
dev.game_control='XOSGameControl';
dev.team='TeamNull';
dev.walk='NewNewNewNewWalk';
dev.kick = 'NewNewKick'

speak = {}
speak.enable = false; 

-- Game Parameters
game = {};
game.teamNumber = 26;
--Not a very clean implementation but we're using this way for now
--Default role: 0 for goalie, 1 for attacker, 2 for defender
--Default team: 0 for blue, 1 for red
if (robotName=='spiffy') then
  game.playerID = 1; --for scarface
  game.role = 1;
elseif (robotName=='pippy') then
  game.playerID = 2; 
  game.role = 1;
else
  game.playerID = 5; 
  game.role = 1;
end
game.teamColor = 0; --Blue team
--game.teamColor = 1; --Red team
game.robotName = robotName;
game.robotID = game.playerID;
game.nPlayers = 2;
--------------------

--FSM and behavior settings
fsm = {};
--SJ: loading FSM config  kills the variable fsm, so should be called first
loadconfig('FSM/Config_XOS_FSM')
fsm.game = 'RoboCup';

fsm.head = {'GeneralPlayer'};
fsm.body = {'GeneralPlayer'};
fsm.playMode = 2; --Go and orbit

--Behavior flags, should be defined in FSM Configs but can be overrided here
--FAST APPROACH TEST
fsm.fast_approach = 1;

-- Team Parameters
team = {};
team.msgTimeout = 5.0;
team.nonAttackerPenalty = 6.0; -- eta sec
team.nonDefenderPenalty = 0.5; -- dist from goal

-- keyframe files
km = {};
km.standup_front = 'km_XOS_StandupFromFront.lua';
km.standup_back = 'km_XOS_StandupFromBack.lua';

-- Low battery level
-- Need to implement this api better...
bat_low = 140; -- 11V warning
bat_med = 145;
gps_only = 0;

-- Use a large fall angle
fallAngle = 60*math.pi/180;

-- Disable fall detection?
sit_disable = 1;
