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

loadconfig('Walk/Config_OP_Walk')
loadconfig('World/Config_OP_World')
loadconfig('Kick/Config_OP_Kick')
loadconfig('Vision/Config_OP_Vision')
loadconfig('Robot/Config_OP_Robot')
--Location Specific Camera Parameters--
loadconfig('Vision/Config_OP_Camera_Grasp')

-- Device Interface Libraries
dev = {};
dev.body = 'OPBody'; 
dev.camera = 'OPCam';
dev.kinematics = 'OPKinematics';
dev.ip_wired = '192.168.123.255';
dev.ip_wireless = '192.168.1.255';
dev.ip_wireless_port = 54321;
dev.game_control='OPGameControl';
dev.team='TeamNSL';
dev.walk='NewNewWalk';
dev.kick = 'NewKick'

-- Game Parameters
game = {};
game.teamNumber = 18;
game.playerID = parse_hostname.get_player_id();
game.robotID = game.playerID;
game.teamColor = parse_hostname.get_team_color();
game.nPlayers = 5;
--------------------
--TODO: playerID based default role setting
game.role = 1; --default attacker
--game.role = 0; --goalie

--FSM and behavior settings
fsm = {};
--SJ: loading FSM config  kills the variable fsm, so should be called first
loadconfig('FSM/Config_WebotsOP_FSM')
fsm.game = 'RoboCup';
fsm.head = {'GeneralPlayer'};
fsm.body = {'GeneralPlayer'};

--Behavior flags, should be defined in FSM Configs but can be overrided here
fsm.enable_obstacle_detection = 1;
fsm.kickoff_wait_enable = 1;
fsm.playMode = 2; --1 for demo, 2 for orbit, 3 for direct approach
fsm.enable_walkkick = 1;
fsm.enable_sidekick = 1;

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

fsm = {};
--fsm.game = 'Dodgeball';
--fsm.game = 'OpDemo'
fsm.game = 'RoboCup';
if( fsm.game == 'RoboCup' ) then
  if (game.playerID == 1) then
    fsm.body = {'OpGoalie'};
    fsm.head = {'OpGoalie'};
  else
    fsm.body = {'OpPlayerNSL'};
    fsm.head = {'OpPlayerNSL'};
  end

  fsm.body = {'OpPlayerNSL'};
  fsm.head = {'OpPlayerNSL'};

elseif( fsm.game == 'Dodgeball' ) then
  fsm.body = {'Dodgeball'};
  fsm.head = {'Dodgeball'};
else
  fsm.body = {'OpDemo'};
  fsm.head = {'OpDemo'};
end

-- Game specific settings
if( fsm.game == 'Dodgeball' ) then
  Config.vision.enable_line_detection = 0;
  Config.vision.enable_midfield_landmark_detection = 0;
end

-- enable obstacle detection

--dev.walk='HZDWalk';

--]]
