module(..., package.seeall);
require('util')
require('vector')

-- Name Platform
platform = {}; 
platform.name = 'WebotsOP'

-- Parameters Files
params = {}
params.name = {"Walk", "World", "Kick", "Vision", "FSM", "Camera"};
params.World_Platform = "OP"
util.LoadConfig(params, platform)

-- Device Interface Libraries
dev = {};
dev.body = 'WebotsOPBody'; 
dev.camera = 'WebotsOPCam';
dev.kinematics = 'OPKinematics';
dev.game_control='WebotsGameControl';
dev.team='TeamNSL';
dev.walk='BasicWalk';
dev.kick='NewKick'; --Extended kick that supports upper body motion

-- Game Parameters
game = {};
game.nPlayers = 3; --5 total robot (including reserve ones)
game.teamNumber = (os.getenv('TEAM_ID') or 0) + 0;
--Webots player id begins at 0 but we use 1 as the first id 
game.playerID = (os.getenv('PLAYER_ID') or 0) + 1;
game.robotID = game.playerID; --For webots, robot ID is the same 
game.role=game.playerID-1; --Default role for webots

--Default team for webots 
if game.teamNumber==0 then  game.teamColor = 0; --Blue team
else game.teamColor = 1; --Red team
end

--FSM and behavior settings
fsm.game = 'RoboCup';
fsm.head = {'GeneralPlayer'};
fsm.body = {'SimplePlayer'};

--Behavior flags, should be defined in FSM Configs but can be overrided here
fsm.enable_obstacle_detection = 1;
fsm.kickoff_wait_enable = 1;
fsm.playMode = 2; --1 for demo, 2 for Play

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
km.standup_front = 'km_OP_StandupFromFront_slow.lua';
km.standup_back = 'km_OP_StandupFromBack_slow.lua';

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
