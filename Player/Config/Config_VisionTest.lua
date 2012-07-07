module(..., package.seeall);
require('util')
require('vector')

-- Name Platform
platform = {}; 
platform.name = 'OP'

util.loadconfig('Walk/Config_WebotsOP_Walk')
util.loadconfig('World/Config_OP_World')
util.loadconfig('Kick/Config_WebotsOP_Kick')
util.loadconfig('Vision/Config_WebotsOP_Vision')
util.loadconfig('Vision/Config_WebotsOP_Camera')

-- Device Interface Libraries
dev = {};
dev.body = 'NullBody'; 
dev.camera = 'ShmCam';
dev.kinematics = 'NullKinematics';
dev.game_control='NullGameControl';
dev.team='TeamNull';
dev.walk='NewWalk';
dev.walk='NewNewWalk'; --New robocup walk that supports walking kicks
dev.kick='NewKick';

-- Game Parameters
game = {};
game.nPlayers = 5; --5 total robot (including reserve ones)
game.teamNumber = (os.getenv('TEAM_ID') or 0) + 0;
--Webots player id begins at 0 but we use 1 as the first id 
game.playerID = (os.getenv('PLAYER_ID') or 0) + 1;
game.robotID = game.playerID; --For webots, robot ID is the same 
game.role=game.playerID-1; --Default role for webots


--Default team (for non-gamecontroller based teamplay)
if game.teamNumber==1 then game.teamColor = 1; --Red team
else game.teamColor = 0; --Blue team
end

--FSM and behavior settings
fsm = {};
--SJ: loading FSM config  kills the variable fsm, so should be called first
util.loadconfig('FSM/Config_WebotsOP_FSM')
fsm.game = 'RoboCup';
fsm.head = {'GeneralPlayer'};
fsm.body = {'GeneralPlayer'};

--Behavior flags, should be defined in FSM Configs but can be overrided here
fsm.enable_obstacle_detection = 1;
fsm.kickoff_wait_enable = 1;
fsm.playMode = 2; --1 for demo, 2 for orbit, 3 for direct approach
fsm.enable_walkkick = 1;
fsm.enable_sidekick = 1;

-------------------------------
fsm.body = {'GeneralPK'};
fsm.playMode = 1;
-------------------------------

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
