module(..., package.seeall);

require('vector')
require('parse_hostname')

platform = {}; 
platform.name = 'Charli'

function loadconfig(configName)
  local localConfig=require(configName);
  for k,v in pairs(localConfig) do
    Config[k]=localConfig[k];
  end
end

--Robot CFG should be loaded first to set PID values
loadconfig('Robot/Config_Charli_Robot') 
loadconfig('Walk/Config_Charli_Walk')
loadconfig('World/Config_Charli_World')
loadconfig('Kick/Config_Charli_Kick')
loadconfig('Vision/Config_Charli_Vision')
--Location Specific Camera Parameters--
loadconfig('Vision/Config_OP_Camera_Grasp')

-- Device Interface Libraries
dev = {};
dev.body = 'CharliBody'; 
dev.camera = 'OPCam';
dev.kinematics = 'CharliKinematics';
dev.ip_wired = '192.168.123.255';
dev.ip_wireless = '192.168.1.255';
dev.ip_wireless_port = 54321;
dev.game_control='OPGameControl';
dev.team='TeamNSL';
--dev.walk='BasicWalk';  --should be updated
dev.walk='NewNewNewNewWalk';
dev.kick = 'NewNewKick'

speak = {}
speak.enable = false; 

-- Game Parameters
game = {};
game.teamNumber = 18;
--Not a very clean implementation but we're using this way for now
local robotName=unix.gethostname();
--Default role: 0 for goalie, 1 for attacker, 2 for defender
--Default team: 0 for blue, 1 for red
game.playerID = 1; --for scarface
game.role = 1; --Default attacker
game.teamColor = 1; --Red team
game.robotName = robotName;
game.robotID = game.playerID;
game.nPlayers = 2;
--------------------

--FSM and behavior settings
fsm = {};
--SJ: loading FSM config  kills the variable fsm, so should be called first
--loadconfig('FSM/Config_OP_FSM')
loadconfig('FSM/Config_Charli_FSM')
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

gps_only = 0;

--Speak enable
speakenable = false;

--Use soft-landing foot trajectory
walk.use_alternative_trajectory = 1;
walk.use_alternative_trajectory = 0;
