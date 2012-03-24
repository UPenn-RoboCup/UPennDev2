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

loadconfig('Walk/Config_XOS_Walk')
loadconfig('World/Config_OP_World')
loadconfig('Kick/Config_XOS_Kick')
loadconfig('Vision/Config_OP_Vision')
loadconfig('Robot/Config_XOS_Robot')

--Location Specific Camera Parameters--
loadconfig('Vision/Config_OP_Camera_Grasp')

-- Device Interface Libraries
dev = {};
dev.body = 'XOSBody'; 
dev.camera = 'OPCam';
dev.kinematics = 'XOSKinematics';
dev.comm='NullComm';
dev.monitor_comm = 'OPMonitorCommWired';
dev.game_control='OPGameControl';
dev.team='TeamNull';
dev.walk='XOSWalk';
dev.kick = 'NewKick'

-- Game Parameters

game = {};
game.teamNumber = 18;
game.playerID = 1;
game.robotID = game.playerID;
game.teamColor = 1;
game.nPlayers = 3;

-- FSM Parameters

fsm = {};
--fsm.game = 'Dodgeball';
fsm.game = 'OpDemo'
--fsm.game = 'RoboCup';
if( fsm.game == 'RoboCup' ) then
--[[
  if (game.playerID == 1) then
    fsm.body = {'OpGoalie'};
    fsm.head = {'OpGoalie'};
  else
    fsm.body = {'OpPlayerNSL'};
    fsm.head = {'OpPlayerNSL'};
  end
--]]

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
BodyFSM = {}
BodyFSM.enable_obstacle_detection = 1;

-- Team Parameters

team = {};
team.msgTimeout = 5.0;
team.nonAttackerPenalty = 6.0; -- eta sec
team.nonDefenderPenalty = 0.5; -- dist from goal

-- keyframe files

km = {};
km.standup_front = 'km_HP_StandupFromFront.lua';
km.standup_back  = 'km_HP_StandupFromBack.lua';
km.kick_right = 'km_HP_kickRight.lua';
km.kick_left = 'km_HP_kickLeft.lua';
km.walk_forward  = 'km_HP_walkForward.lua';
km.walk_backward  = 'km_HP_walkBackward.lua';

-- Low battery level
-- Need to implement this api better...
bat_low = 100; -- 10V warning


speedFactor = 1.0; --all SM work in real time
webots_vision = 0; --use full vision
