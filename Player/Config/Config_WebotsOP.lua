module(..., package.seeall);
require('vector')

-- Name Platform
platform = {}; 
platform.name = 'WebotsOP'

function loadconfig(configName)
  local localConfig=require(configName);
  for k,v in pairs(localConfig) do
    Config[k]=localConfig[k];
  end
end

loadconfig('Walk/Config_WebotsOP_Walk')
loadconfig('World/Config_OP_World')
loadconfig('Kick/Config_WebotsOP_Kick')
--loadconfig('Kick/Config_WebotsOP_KickPunch')
loadconfig('Vision/Config_WebotsOP_Vision')

--Location Specific Camera Parameters--
loadconfig('Vision/Config_WebotsOP_Camera')

-- Device Interface Libraries
dev = {};
dev.body = 'WebotsOPBody'; 
dev.camera = 'WebotsOPCam';
dev.kinematics = 'OPKinematics';
dev.game_control='WebotsGameControl';
dev.team='TeamNSL';
dev.walk='NewWalk';
dev.walk='NewNewWalk'; --New robocup walk that supports walking kicks
--dev.walk='BoxWalk'; --New walk that supports different foot stance
dev.kick='NewKick';
--dev.kick='NSLKickPunch'; --Extended kick that supports upper body motion

-- Game Parameters
game = {};
game.teamNumber = (os.getenv('TEAM_ID') or 0) + 0;
--Webots player id begins at 0 but we use 1 as the first id 
game.playerID = (os.getenv('PLAYER_ID') or 0) + 1;
game.robotID = game.playerID;
if game.playerID==1 then game.role = 4; end --Goalie handling 
if game.teamNumber==1 then game.teamColor = 1; --Red team
else game.teamColor = 0; --Blue team
end

game.nPlayers = 4;

--FSM and behavior settings
fsm = {};
--SJ: loading FSM config  kills the variable fsm, so should be called first
loadconfig('FSM/Config_WebotsOP_FSM')

fsm.game = 'Stretcher';

-- Set the Body and Head FSMs based on GameFSM
fsm.body = {fsm.game};
fsm.head = {fsm.game};
--[[
fsm.head = {'GeneralPlayer'};
fsm.body = {'GeneralPlayer'};
--]]

-- Game specific settings
if( fsm.game == 'Dodgeball' ) then
  Config.vision.enable_line_detection = 0;
  Config.vision.enable_midfield_landmark_detection = 0;
end


if( fsm.game == 'Stretcher' ) then
  loadconfig( 'Config_Stretcher' );
  game.teamNumber = 18;
  game.playerID = 1;
end


--Behavior flags, defined in FSM Configs and can be overrided here
fsm.enable_obstacle_detection = 1;
fsm.playMode = 3; --1 for demo, 2 for orbit, 3 for direct approach
fsm.enable_walkkick = 1;

-- Team Parameters
team = {};
team.msgTimeout = 5.0;
team.nonAttackerPenalty = 6.0; -- eta sec
team.nonDefenderPenalty = 0.5; -- dist from goal

-- keyframe files
km = {};
km.standup_front = 'km_NSLOP_StandupFromFront.lua';
km.standup_back = 'km_NSLOP_StandupFromBack.lua';

--Sitting parameters
sit={};
sit.bodyHeight=0.20; --Fixed for webots
sit.supportX=-0.010;
sit.bodyTilt=5*math.pi/180;
sit.dpLimit=vector.new({.1,.01,.03,.1,.3,.1});
sit.dpLimit=vector.new({.1,.01,.06,.1,.3,.1});--Faster sit

--Standing parameters
stance={};
stance.dpLimit=vector.new({.04, .03, .04, .4, .4, .4});
stance.dpLimit=vector.new({.04, .03, .07, .4, .4, .4});--Faster standup

-- Head Parameters
head = {};
head.camOffsetZ = 0.37;
head.pitchMin = -35*math.pi/180;
head.pitchMax = 68*math.pi/180;
head.yawMin = -90*math.pi/180;
head.yawMax = 90*math.pi/180;
head.cameraPos = {{0.05, 0.0, 0.05}} --OP, spec value, may need to be recalibrated
head.cameraAngle = {{0.0, 0.0, 0.0}}; --Default value for production OP
head.neckZ=0.0765; --From CoM to neck joint 
head.neckX=0.013; --From CoM to neck joint
head.bodyTilt = 0;

--km.kick_right = 'km_NSLOP_taunt1.lua';
--km.kick_left = 'km_NSLOP_StandupFromFront2.lua';


--SJ: I separated non-robocup FSMs here
--Dodgeball FSM
--[[
loadconfig('FSM/Config_WebotsOP_FSM')
fsm.game = 'Dodgeball';
fsm.head = {'GeneralPlayer'};
fsm.body = {'GeneralPlayer'};
Config.vision.enable_line_detection = 0;
Config.vision.enable_midfield_landmark_detection = 0;
--]]

--Stretcher FSM
--[[
loadconfig('FSM/Config_WebotsOP_FSM')
loadconfig( 'Config_Stretcher' );
game.teamNumber = 18;
game.playerID = 1;
fsm.game = 'Stretcher';
fsm.head = {'GeneralPlayer'};
fsm.body = {'GeneralPlayer'};
Config.vision.enable_line_detection = 0;
Config.vision.enable_midfield_landmark_detection = 0;
--]]
