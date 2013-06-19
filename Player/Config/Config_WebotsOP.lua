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
--dev.team='TeamNSL';
dev.team='TeamGeneral';
dev.ip_wired = '192.168.123.255';
dev.ip_wired_port = 54321;
dev.ip_wireless = '192.168.1.255'; --Our Router
dev.ip_wireless_port = 54321;
dev.walk='EvenBetterWalk'; --Walk with generalized walkkick definitions
--dev.walk='CleanWalk';
dev.walk='AwesomeWalk';
dev.kick='PunchKick'; --Extended kick that supports upper body motion

largestep_enable = false;
largestep_enable = true;
dev.largestep = 'ZMPStepKick';--ZMP Preview motion 


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

--FAST APPROACH TEST
fsm.fast_approach = 0;
--fsm.bodyApproach.maxStep = 0.06;

--1 for randomly doing evade kick
--2 for using obstacle information
--fsm.enable_evade = 0;
--fsm.enable_evade = 1;--Randomly do evade kick
fsm.enable_evade = 2;--Do evade kick when obstructed

-- Team Parameters
team = {};
team.msgTimeout = 5.0;
team.tKickOffWear =7.0;

team.walkSpeed = 0.25; --Average walking speed 
team.turnSpeed = 2.0; --Average turning time for 360 deg
team.ballLostPenalty = 4.0; --ETA penalty per ball loss time
team.fallDownPenalty = 4.0; --ETA penalty per ball loss time
team.nonAttackerPenalty = 0.8; -- distance penalty from ball
team.nonDefenderPenalty = 0.5; -- distance penalty from goal
team.force_defender = 0;--Enable this to force defender mode
team.test_teamplay = 0; --Enable this to immobilize attacker to test team behavior

--if ball is away than this from our goal, go support
team.support_dist = 3.0; 
team.supportPenalty = 0.5; --dist from goal
team.use_team_ball = 0;
team.team_ball_timeout = 3.0;  --use team ball info after this delay
team.team_ball_threshold = 0.5;

team.avoid_own_team = 1;
team.avoid_other_team = 1;

team.flip_correction = 1;
team.flip_threshold_x = 3.0;
team.flip_threshold_y = 3.0;




-- Keyframe files
km = {};
km.standup_front = 'km_NSLOP_StandupFromFront.lua';
km.standup_back = 'km_NSLOP_StandupFromBack.lua';
km.standup_back2 = 'km_NSLOP_StandupFromBack3.lua';

goalie_dive = 1; --1 for arm only, 2 for actual diving
goalie_dive_waittime = 6.0; --How long does goalie lie down?
--fsm.goalie_type = 1;--moving/move+stop/stop+dive/stop+dive+move
--fsm.goalie_type = 2;--moving/move+stop/stop+dive/stop+dive+move
fsm.goalie_type = 3;--moving/move+stop/stop+dive/stop+dive+move
--fsm.goalie_type = 4;--moving/move+stop/stop+dive/stop+dive+move

--fsm.goalie_reposition=0; --No reposition except for clearing the ball
--fsm.goalie_reposition=1; --Yaw reposition
--fsm.goalie_reposition=2; --Position reposition
fsm.goalie_reposition=3; --No reposition at all (for testing)



fsm.bodyAnticipate.thFar = {0.4,0.4,30*math.pi/180};
fsm.goalie_use_walkkick = 1;--should goalie use walkkick or long kick?

--Diving detection parameters
fsm.bodyAnticipate.timeout = 3.0;
fsm.bodyAnticipate.center_dive_threshold_y = 0.05; 
fsm.bodyAnticipate.dive_threshold_y = 1.0;
fsm.bodyAnticipate.ball_velocity_th = 1.0; --min velocity for diving
fsm.bodyAnticipate.ball_velocity_thx = -1.0; --min x velocity for diving
fsm.bodyAnticipate.rCloseDive = 2.0; --ball distance threshold for diving

-- Low battery level
bat_med = 122; -- Slow down if voltage drops below 12.2V 
bat_low = 118; -- 11.8V warning
batt_max = 120; --only do rollback getup when battery is enough
use_rollback_getup = 0;
	
--Fall check
fallAngle = 40*math.pi/180;
falling_timeout = 0.3;

listen_monitor = 1;
-- Shutdown Vision and use ground truth gps info only
--Now auto-detect from 3rd parameter
use_gps_only = tonumber(os.getenv('USEGPS')) or 0;
print("GPS:",use_gps_only)

------------------------------------
-- Game-type Specific Configurations
------------------------------------

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


--Enable this to immobilize attacker to test team behavior
team.test_teamplay = 0; 

world.use_new_goalposts = 1;
world.triangulation_threshold = 4.0; 
world.angle_update_threshold = 1.0;
team.flip_correction = 1;
world.position_update_threshold = 4.5;--Goalie position shouldn't move 




vision.enable_corner_detection = 1;

--fsm.playMode = 1;--Demo testing
min_eta_look = 1.0; 
