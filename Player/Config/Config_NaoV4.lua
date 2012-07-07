module(..., package.seeall);
require('util')
require('vector')
require('parse_hostname')

platform = {};
platform.name = 'NaoV4'

params = {}
params.name = {"Walk", "World", "Kick", "Vision", "FSM", "Camera"};
params.World_Platform = "Nao"
params.Walk = 'FastStable' 
params.Camera = 'Grasp'

util.LoadConfig(params, platform)

listen_monitor=1
  
-- Game Parameters

game = {};
game.teamNumber = 25;
game.playerID = parse_hostname.get_player_id();
game.robotID = game.playerID;
game.teamColor = parse_hostname.get_team_color();
game.role = game.playerID-1; -- 0 for goalie
game.nPlayers = 4;

-- Devive Interface Libraries
dev = {};
dev.body = 'NaoBody'; 
dev.camera = 'NaoCam';
dev.kinematics = 'NaoKinematics';
dev.ip_wired = '192.168.69.255';
--dev.ip_wireless = '192.168.69.255';
dev.ip_wireless = '192.168.255.255';
dev.game_control = 'NaoGameControl';
dev.team='TeamSPL';
dev.walk = 'Walk/NaoV4Walk';
dev.kick = 'NewKick';

--Speak enable
speakenable = 1;


-- FSM Parameters
fsm.game = 'RoboCup';
if game.role == 0 then
  fsm.body = {'NaoGoalie'}
else
  fsm.body = {'NaoKickLogic'};
  --fsm.body = {'GeneralPlayer'};
end
fsm.head = {'NaoPlayer'};
--fsm.head = {'GeneralPlayer'};

-- Team Parameters

team = {};
team.msgTimeout = 5.0;
team.nonAttackerPenalty = 6.0; -- eta sec
team.nonDefenderPenalty = 0.5; -- dist from goal


head = {};
head.camOffsetZ = 0.41;
head.pitchMin = -35*math.pi/180;
head.pitchMax = 30*math.pi/180;
head.yawMin = -120*math.pi/180;
head.yawMax = 120*math.pi/180;
--Update with naoV4 camera values
head.cameraPos = {{0.05871, 0.0, 0.06364},
                  {0.05071, 0.0, 0.01774}}; 
head.cameraAngle = {{0.0, 1.2*math.pi/180, 0.0},
                    {0.0, 39.7*math.pi/180, 0.0}};

head.neckZ=0.14; --From CoM to neck joint
head.neckX=0;  
head.bodyTilt = 0;

-- keyframe files
km = {};
km.standup_front = 'km_NaoV4_StandupFromFront.lua';
km.standup_back = 'km_NaoV4_StandupFromBack.lua';
km.time_to_stand = 30; -- average time it takes to stand up in seconds

--Sit/stand stance parameters
stance={};
stance.bodyHeightSit = 0.18;
stance.supportXSit = 0.020;
stance.bodyHeightDive= 0.25;
stance.dpLimitSit=vector.new({.1,.01,.06,.1,.3,.1});
stance.bodyTiltStance=0*math.pi/180; --bodyInitial bodyTilt, 0 for webots
stance.dpLimitStance=vector.new({.04, .03, .06, .05, .4, .1});
stance.delay = 80; --amount of time to stand still after standing to regain balance.

--For compatibility with OP
--Should be more generally handled in Body..
servo={};
servo.pid=0;
