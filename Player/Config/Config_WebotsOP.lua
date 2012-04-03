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
--dev.team='TeamSPL';
dev.walk='NewWalk';
dev.walk='NewNewWalk'; --New robocup walk that supports walking kicks
--dev.walk='BoxWalk'; --New walk that supports different foot stance
--dev.kick='NewKick';
dev.kick='NewNewKick'; --Extended kick that supports upper body motion

-- Game Parameters
game = {};
game.nPlayers = 5; --5 total robot (including reserve ones)
--Should be 4 robostadium nao gamecontroller
game.nPlayers = 4; 

game.teamNumber = (os.getenv('TEAM_ID') or 0) + 0;
--Webots player id begins at 0 but we use 1 as the first id 
game.playerID = (os.getenv('PLAYER_ID') or 0) + 1;
game.robotID = game.playerID; --For webots, robot ID is the same 
game.role=game.playerID-1; --Default role for webots

--Default team for webots 
if game.teamNumber==0 then  game.teamColor = 0; --Blue team
else game.teamColor = 0; --Red team
end

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

--[[
--Enable these for penalty-kick
dev.team='TeamNull'; --Turn off teamplay for challenges
fsm.body = {'GeneralPK'};
fsm.playMode = 2;
--]]

--[[
--Enable this for throw-in 
--fsm.body = {'ThrowInChallenge'};
--]]

--Enable this for double pass
--[[
fsm.body={'DoublePassChallenge'};
dev.team='TeamDoublePass';
--]]


-- Team Parameters
team = {};
team.msgTimeout = 5.0;
team.nonAttackerPenalty = 6.0; -- eta sec
team.nonDefenderPenalty = 0.5; -- dist from goal

-------------------------------------
-- Robot specific parameters
-------------------------------------

-- keyframe files
km = {};
km.standup_front = 'km_NSLOP_StandupFromFront.lua';
km.standup_back = 'km_NSLOP_StandupFromBack.lua';

--Sit/stand stance parameters
stance={};
stance.bodyHeightSit = 0.20;
stance.supportXSit = -0.010;
stance.bodyHeightDive= 0.25;
stance.bodyTiltStance=0*math.pi/180; --bodyInitial bodyTilt, 0 for webots
stance.dpLimitStance=vector.new({.04, .03, .07, .4, .4, .4});
stance.dpLimitSit=vector.new({.1,.01,.06,.1,.3,.1});

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
fsm.head = {'Stretcher'};
fsm.body = {'Stretcher'};
Config.vision.enable_line_detection = 0;
Config.vision.enable_midfield_landmark_detection = 0;
--]]
