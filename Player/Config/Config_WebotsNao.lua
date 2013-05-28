module(..., package.seeall);
require('util')
require('parse_hostname')
require('vector')
require('os')

platform = {};
platform.name = 'WebotsNao'

listen_monitor = 1

webots = 1

-- Parameters Files
params = {}
params.name = {"Walk", "World", "Kick", "Vision", "FSM", "Camera"};
util.LoadConfig(params, platform)

params.world = 'World/Config_WebotsNao_World'
params.walk = 'Walk/Config_WebotsNao_Walk' 
params.kick = 'Kick/Config_WebotsNao_Kick'
params.vision = 'Vision/Config_WebotsNao_Vision'
params.camera = 'Vision/Config_WebotsNao_Camera'
params.fsm = 'FSM/Config_WebotsNao_FSM'

-- Device Interface Libraries
dev = {};
dev.body = 'NaoWebotsBody'; 
dev.camera = 'NaoWebotsCam';
dev.kinematics = 'NaoWebotsKinematics';
dev.game_control='WebotsGameControl';
dev.team= 'TeamSPL';
dev.kick = 'BasicKick';
--dev.walk = 'Walk/NaoV4Walk';
dev.walk = 'EvenBetterWalk';

-- Game Parameters

game = {};
game.teamNumber = (os.getenv('TEAM_ID') or 0) + 0;
-- webots player ids begin at 0 but we use 1 as the first id
game.playerID = (os.getenv('PLAYER_ID') or 0) + 1;
game.robotID = game.playerID;
game.role = game.playerID-1; -- default role, 0 for goalie 
game.nPlayers = 4;

-- Shutdown Vision and use ground truth gps info only
--Now auto-detect from 3rd parameter
use_gps_only = tonumber(os.getenv('USEGPS')) or 0;
print("GPS:",use_gps_only)




--To handle non-gamecontroller-based team handling for webots
if game.teamNumber==0 then game.teamColor = 0; --Blue team
else game.teamColor = 1; --Red team
end

fsm.game = 'RoboCup';
--fsm.body = {'NaoKickLogic'};
fsm.body = {'GeneralPlayer'};

fsm.head = {'GeneralPlayer'};


--fsm.head = {'NaoPlayer'};



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


-- keyframe files

km = {};
km.kick_right = 'km_WebotsNao_KickForwardRight.lua';
km.kick_left = 'km_WebotsNao_KickForwardLeft.lua';
km.standup_front = 'km_WebotsNao_StandupFromFront.lua';
km.standup_back = 'km_WebotsNao_StandupFromBack.lua';


km.standup_front = 'km_WebotsNao_StandupFromFront.lua';
km.standup_back = 'km_WebotsNao_StandupFromBack.lua';
km.time_to_stand = 30; -- average time it takes to stand up in seconds



--Sit/stand stance parameters
stance={};
stance.bodyHeightSit = 0.225;
stance.supportXSit = 0;
stance.dpLimitSit=vector.new({.1,.01,.03,.1,.3,.1});
stance.bodyHeightDive= 0.25;
stance.bodyTiltStance=0*math.pi/180; --bodyInitial bodyTilt, 0 for webots
stance.dpLimitStance = vector.new({.04, .03, .04, .05, .4, .1});
stance.delay = 80; --amount of time to stand still after standing to regain balance.





goalie_dive = 2; --1 for arm only, 2 for actual diving
goalie_dive_waittime = 6.0; --How long does goalie lie down?
--fsm.goalie_type = 1;--moving/move+stop/stop+dive/stop+dive+move
--fsm.goalie_type = 2;--moving/move+stop/stop+dive/stop+dive+move
fsm.goalie_type = 3;--moving/move+stop/stop+dive/stop+dive+move
--fsm.goalie_type = 4;--moving/move+stop/stop+dive/stop+dive+move
--fsm.goalie_reposition=0; --No reposition
fsm.goalie_reposition=1; --Yaw reposition
--fsm.goalie_reposition=2; --Position reposition
fsm.bodyAnticipate.thFar = {0.4,0.4,30*math.pi/180};
fsm.goalie_use_walkkick = 1;--should goalie use walkkick or long kick?

--Diving detection parameters
fsm.bodyAnticipate.timeout = 3.0;
fsm.bodyAnticipate.center_dive_threshold_y = 0.05; 
fsm.bodyAnticipate.dive_threshold_y = 1.0;
fsm.bodyAnticipate.ball_velocity_th = 1.0; --min velocity for diving
fsm.bodyAnticipate.ball_velocity_thx = -1.0; --min x velocity for diving
fsm.bodyAnticipate.rCloseDive = 2.0; --ball distance threshold for diving




--Dummy variables
bat_low = 999;
bat_med = 999;
