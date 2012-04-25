module(..., package.seeall);

require('vector')
require('parse_hostname')

platform = {}; 
platform.name = 'OP'

function loadconfig(configName)
  local localConfig=require(configName);
  for k,v in pairs(localConfig) do
    Config[k]=localConfig[k];
  end
end

--Robot CFG should be loaded first to set PID values
loadconfig('Robot/Config_OP_Robot') 

--loadconfig('Walk/Config_OP_Walk')
loadconfig('Walk/Config_OP_Walk_Basic')
loadconfig('World/Config_OP_World')
loadconfig('Kick/Config_OP_Kick')
loadconfig('Vision/Config_OP_Vision')
--Location Specific Camera Parameters--
loadconfig('Vision/Config_OP_Camera_USOpen_Bowdoin')

-- Device Interface Libraries
dev = {};
dev.body = 'OPBody'; 
dev.camera = 'OPCam';
dev.kinematics = 'OPKinematics';
dev.ip_wired = '192.168.123.255';
dev.ip_wireless = '139.140.218.255';
dev.game_control='NaoGameControl';
dev.team='TeamNSL';
--dev.walk='BasicWalk';  --should be updated
dev.walk='NewNewNewWalk';
dev.kick = 'NewNewKick'

speak = {}
speak.enable = false 

-- Game Parameters
game = {};
game.teamNumber = 18;
--Not a very clean implementation but we're using this way for now
local robotName=unix.gethostname();
--Default role: 0 for goalie, 1 for attacker, 2 for defender
--Default team: 0 for blue, 1 for red
if (robotName=='scarface') then
  game.playerID = 1; --for scarface
  game.role = 1; --Default attacker
elseif (robotName=='linus') then
  game.playerID = 2; 
  game.role = 0; --Default attacker
elseif (robotName=='betty') then
  game.playerID = 3; 
  game.role = 1; --Default attacker
elseif (robotName=='lucy') then
  game.playerID = 4; 
  game.role = 1; --Default attacker
else
  game.playerID = 5; 
  game.role = 1; --Default attacker
end
--game.teamColor = 0; --Blue team
game.teamColor = 1; --Red team
game.robotName = robotName;
game.robotID = game.playerID;
game.nPlayers = 5;
--------------------

--FSM and behavior settings
fsm = {};
--SJ: loading FSM config  kills the variable fsm, so should be called first
--loadconfig('FSM/Config_OP_FSM')
loadconfig('FSM/Config_OP_FSM')
fsm.game = 'RoboCup';
fsm.head = {'GeneralPlayer'};
fsm.body = {'GeneralPlayer'};

--Behavior flags, should be defined in FSM Configs but can be overrided here
fsm.enable_obstacle_detection = 1;
fsm.kickoff_wait_enable = 0;
--fsm.playMode = 3; --1 for demo, 2 for orbit, 3 for direct approach
--fsm.enable_walkkick = 1;
--fsm.enable_sidekick = 1;

fsm.playMode = 3; --1 for demo, 2 for orbit, 3 for direct approach
fsm.enable_walkkick = 0;
fsm.enable_sidekick = 0;

--FAST APPROACH TEST
fsm.fast_approach = 1;
--fsm.bodyApproach.maxStep = 0.06;

-- Team Parameters
team = {};
team.msgTimeout = 5.0;
team.nonAttackerPenalty = 6.0; -- eta sec
team.nonDefenderPenalty = 0.5; -- dist from goal

-- keyframe files
km = {};
km.standup_front = 'km_NSLOP_StandupFromFront.lua';
km.standup_back = 'km_NSLOP_StandupFromBack.lua';

-- Low battery level
-- Need to implement this api better...
bat_low = 100; -- 10V warning
--[[
-- Stretcher
loadconfig( 'Config_Stretcher' );
game.playerID = 1;
fsm.game = 'Stretcher';
fsm.head = {'Stretcher'};
fsm.body = {'Stretcher'};
--]]
gps_only = 0;
