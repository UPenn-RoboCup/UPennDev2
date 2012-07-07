module(..., package.seeall);
require('util')
require('parse_hostname')
require('vector')
require('os')

platform = {};
platform.name = 'WebotsNao'

-- Parameters Files
params = {}
params.name = {"Walk", "World", "Kick", "Vision", "FSM", "Camera"};
params.World_Platform = "OP"
util.LoadConfig(params, platform)

-- Device Interface Libraries
dev = {};
dev.body = 'GenericWebotsBody'; 
dev.camera = 'NaoWebotsCam';
dev.kinematics = 'GenericKinematics';
dev.game_control = 'WebotsNaoGameControl';
dev.team='TeamNull';
dev.walk = 'NaoWalk';
dev.kick = 'NaoKick';

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
fsm.game = 'RoboCup';
if (game.playerID == 1) then
  fsm.body = {'NaoGoalie'};
  fsm.head = {'NaoGoalie'};
else
--[[
  fsm.body = {'NaoPlayer'};
  fsm.head = {'NaoPlayer'};
--]]
  fsm.body = {'OpPlayer'};
  fsm.head = {'NaoPlayer'};
end

--For testing some new stuff
--[[
dev.walk = 'NSLWalk';
dev.kick = 'NaoKick';
fsm.body = {'OpPlayerRobocup'};
--]]

-- Team Parameters

team = {};
team.msgTimeout = 5.0;
team.nonAttackerPenalty = 6.0; -- eta sec
team.nonDefenderPenalty = 0.5; -- dist from goal


--Head Parameters

head = {};
head.camOffsetZ = 0.41;
head.pitchMin = -35*math.pi/180;
head.pitchMax = 30*math.pi/180;
head.yawMin = -120*math.pi/180;
head.yawMax = 120*math.pi/180;
head.cameraPos = {{0.05390, 0.0, 0.06790},
                  {0.04880, 0.0, 0.02381}}; 
head.cameraAngle = {{0.0, 0.0, 0.0},
                    {0.0, 40*math.pi/180, 0.0}};
head.neckZ=0.14; --From CoM to neck joint
head.neckX=0;  

--For generic
head.neckZ=0.24; --From CoM to neck joint



-- keyframe files

km = {};
km.kick_right = 'km_WebotsNao_KickForwardRight.lua';
km.kick_left = 'km_WebotsNao_KickForwardLeft.lua';
km.standup_front = 'km_WebotsNao_StandupFromFront.lua';
km.standup_back = 'km_WebotsNao_StandupFromBack.lua';


-- sitting parameters

sit = {};
sit.bodyHeight = 0.22;
sit.supportX = 0;
sit.dpLimit = vector.new({.1,.01,.03,.1,.3,.1});


-- standing parameters

stance = {};
stance.dpLimit = vector.new({.04, .03, .04, .05, .4, .1});
stance.delay = 80; --amount of time to stand still after standing to regain balance.



-- enable obstacle detection
BodyFSM = {}
BodyFSM.enable_obstacle_detection = 1;

