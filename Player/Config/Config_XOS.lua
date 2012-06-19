module(..., package.seeall);

require('vector')

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
loadconfig('Vision/Config_XOS_Camera_VT')

-- Device Interface Libraries
dev = {};
dev.body = 'XOSBody'; 
dev.camera = 'XOSCam';
dev.kinematics = 'XOSKinematics';
dev.ip_wired = '192.168.123.255';
dev.ip_wireless = '255.255.255.255';
dev.ip_wireless_port = 3838;
dev.game_control='XOSGameControl';
dev.team='TeamXOS';
dev.walk='NewNewNewNewWalk';
dev.kick = 'NewNewKick'

speak = {}
speak.enable = false; 

-- Game Parameters
game = {};
game.teamNumber = 5;
--Not a very clean implementation but we're using this way for now
local robotName=unix.gethostname();
--Default role: 0 for goalie, 1 for attacker, 2 for defender
--Default team: 0 for blue, 1 for red
if (robotName=='hp1') then
  game.playerID = 1; --for scarface
  game.role = 1; --Default attacker
elseif (robotName=='hp2') then
  game.playerID = 2; 
  game.role = 0; --Default goalie
else
  game.playerID = 5; 
  game.role = 1; --Default attacker
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

--Behavior flags, should be defined in FSM Configs but can be overrided here
fsm.enable_obstacle_detection = 1;
fsm.kickoff_wait_enable = 0;
fsm.playMode = 2; --1 for demo, 2 for orbit, 3 for direct approach
fsm.enable_walkkick = 0;
fsm.enable_sidekick = 0;

--FAST APPROACH TEST
fsm.fast_approach = 0;
--fsm.bodyApproach.maxStep = 0.06;

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
bat_low = 110; -- 11V warning

--[[
-- Stretcher
loadconfig( 'Config_Stretcher' );
game.playerID = 1;
fsm.game = 'Stretcher';
fsm.head = {'Stretcher'};
fsm.body = {'Stretcher'};
dev.team = "TeamPrimeQ"
dev.walk = "StretcherWalk"
--]]

gps_only = 0;

--Speak enable
speakenable = false;

--VT goalposts are thicker
world.postDiameter = 0.12;

--Slow down max speed
fsm.bodyPosition.maxStep3 = 0.06;
