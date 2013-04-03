module(..., package.seeall);
require('util')
require('vector')

-- Name Platform
platform = {}; 
platform.name = 'WebotsOP'

-- Parameters Files
params = {}
params.name = {"Walk", "World", "Kick", "Vision", "FSM", "Camera"};
util.LoadConfig(params, platform)

-- Device Interface Libraries
dev = {};
dev.body = 'WebotsOPBody'; 
dev.camera = 'WebotsOPCam';
dev.kinematics = 'OPKinematics';
dev.game_control='WebotsGameControl';
dev.team='TeamNSL';
dev.ip_wired = '192.168.123.255';
dev.ip_wired_port = 54321;
dev.ip_wireless = '192.168.1.255'; --Our Router
dev.ip_wireless_port = 54321;
dev.walk='EvenBetterWalk'; --Walk with generalized walkkick definitions
dev.kick='NewNewKick'; --Extended kick that supports upper body motion

-- disable speak for webots which causes lua crash with error if espeak not installed
speakenable = 0

-- Head Parameters
head = {};
head.camOffsetZ = 0.37;
head.pitchMin = -35*math.pi/180;
head.pitchMax = 58*math.pi/180;
head.yawMin = -90*math.pi/180;
head.yawMax = 90*math.pi/180;
head.cameraPos = {{0.05, 0.0, 0.05}} --OP, spec value, may need to be recalibrated
head.cameraAngle = {{0.0, 0.0, 0.0}}; --Default value for production OP
head.neckZ=0.0765; --From CoM to neck joint 
head.neckX=0.013; --From CoM to neck joint
head.bodyTilt = 0;

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
else game.teamColor = 1; --Red team
end

-- FSM and Behavior flags, should be defined in FSM Configs but can be overridden here
fsm.game = 'RoboCup';
fsm.head = {'GeneralPlayer'};
fsm.body = {'GeneralPlayer'};

-- Keyframe files
km = {};
km.standup_front = 'km_NSLOP_StandupFromFront.lua';
km.standup_back = 'km_NSLOP_StandupFromBack.lua';
km.standup_back2 = 'km_NSLOP_StandupFromBack3.lua';
--km.standup_back = 'km_NSLOP_StandupFromBack3.lua';
--km.kick_right = 'km_NSLOP_taunt1.lua';
--km.kick_left = 'km_NSLOP_StandupFromFront2.lua';

goalie_dive = 2; --1 for arm only, 2 for actual diving
goalie_dive_waittime = 6.0; --How long does goalie lie down?

-- Low battery level
bat_med = 122; -- Slow down if voltage drops below 12.2V 
bat_low = 118; -- 11.8V warning
batt_max = 120; --only do rollback getup when battery is enough
use_rollback_getup = 0;
	
--Fall check
fallAngle = 40*math.pi/180;
falling_timeout = 0.3;

-- Shutdown Vision and use ground truth gps info only
use_gps_only = 0;
--use_gps_only = 1;

------------------------------------
-- Game-type Specific Configurations
------------------------------------

-- Play Football
--[[
fsm.game = 'Football';
fsm.head = {'Football'};
fsm.body = {'Football'};
dev.team = 'TeamFootball'
--]]

--[[
--Enable these for penalty-kick
dev.team='TeamNull'; --Turn off teamplay for challenges
fsm.body = {'GeneralPK'};
--]]

--[[
--Enable this for throw-in 
dev.team='TeamNull'; --Turn off teamplay for challenges
fsm.body = {'ThrowInChallenge'};
--]]

-- Doublepass challenge
--[[
dev.team='TeamDoublePass';
fsm.body={'DoublePassChallenge'};
fsm.headTrack.timeout = 2.0 * speedFactor;
fsm.headTrack.tLost = 1.5 * speedFactor;
fsm.headTrack.minDist = 0.15; --Default value 0.30,If ball is closer than this, don't look up
--]]

-- Obstacle Avoidance Challenge
--[[
fsm.head = {'ObstacleChallenge'};
fsm.body = {'ObstacleChallenge'};
fsm.avoidance_mode = 1 -- ball dribble
fsm.avoidance_mode = 0 -- walk towards goal, no ball 
fsm.avoidance_mode = 2 -- Potential Field based navigation
--fsm.avoidance_mode = 3 -- Potential Field based Dribble
obs_challenge = 1;
obs_challenge = 0;
fsm.enable_sidekick = 1;
fsm.thSideKick1 = 30*math.pi/180;
fsm.thSideKick2 = 135*math.pi/180;
fsm.thDistSideKick = 1.0;
--]]
