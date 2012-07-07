module(..., package.seeall);
require('util')
require('parse_hostname')
require('vector')
require('os')

platform = {};
platform.name = 'WebotsSaffir'

util.loadconfig('Walk/Config_WebotsSaffir_Walk')
util.loadconfig('Kick/Config_WebotsSaffir_Kick')
util.loadconfig('World/Config_Charli_World')
util.loadconfig('Vision/Config_WebotsOP_Vision')

--Location Specific Camera Parameters--
util.loadconfig('Vision/Config_WebotsOP_Camera')

-- Device Interface Libraries
dev = {};
--dev.body = 'WebotsSaffirBody'; 
dev.body = 'WebotsSaffirPennBody'; 
dev.camera = 'WebotsOPCam';
dev.kinematics = 'SaffirKinematics';
dev.comm = 'WebotsNaoComm';
dev.monitor_comm = 'NullComm';
dev.game_control='WebotsGameControl';
dev.team='TeamNull';

dev.walk = 'NewWalk';
dev.kick = 'NewKick';
--dev.walk = 'HTWalk';
dev.walk = 'Run';
--dev.walk = 'BasicWalk';


-- Game Parameters
game = {};
game.teamNumber = (os.getenv('TEAM_ID') or 0) + 0;
-- webots player ids begin at 0 but we use 1 as the first id
game.playerID = (os.getenv('PLAYER_ID') or 0) + 1;
game.robotID = game.playerID;
game.teamColor = 1;
game.nPlayers = 4;


-- FSM Parameters
fsm = {};
util.loadconfig('FSM/Config_WebotsCharli_FSM')

fsm.game = 'OpDemo'
fsm.body = {'GeneralPlayer'};
fsm.head = {'GeneralPlayer'};

-- Team Parameters

team = {};
team.msgTimeout = 5.0;
team.nonAttackerPenalty = 6.0; -- eta sec
team.nonDefenderPenalty = 0.5; -- dist from goal


--Head Parameters

head = {};
head.camOffsetZ = 0.41;
head.pitchMin = -35*math.pi/180;
head.pitchMax = 68*math.pi/180;
head.yawMin = -120*math.pi/180;
head.yawMax = 120*math.pi/180;
head.cameraPos = {{0.05, 0.0, 0.05}} --OP, spec value, may need to be recalibrated
head.cameraAngle = {{0.0, 0.0, 0.0}}; --Default value for production OP
head.neckZ=0.15; --From CoM to neck joint , Hubo prototype
head.neckX=0.03; --From CoM to neck joint , Hubo prototype

-- keyframe files

km = {};
km.standup_front = 'km_Saffir_StandupFromFront.lua';
km.standup_back = 'km_Saffir_StandupFromBack.lua';

--Sit/stand stance parameters
stance={};
stance.bodyHeightSit = 0.40;
stance.supportXSit = -0.00;
stance.bodyHeightDive= 0.65;
stance.bodyTiltStance=0*math.pi/180; --bodyInitial bodyTilt, 0 for webots
stance.dpLimitStance = vector.new({.4, .3, .4, .05, .4, .1})*0.6;
stance.dpLimitStance=vector.new({.04, .03, .07, .4, .4, .4});
stance.dpLimitSit=vector.new({.1,.01,.06,.1,.3,.1})*2;
stance.delay = 80; 

-- enable obstacle detection
BodyFSM = {}
BodyFSM.enable_obstacle_detection = 1;

--How slow is the walking compared to real OP?
speedFactor = 4.0;

--Skip all checks in vision for 160*120 image 
webots_vision = 1; 
