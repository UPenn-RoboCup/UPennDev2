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
--loadconfig('World/Config_OP_World')
loadconfig('World/Config_WebotsOP_World')
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
dev.team='TeamPrimeQ';

dev.ip_wireless = '192.168.1.255'; --Our Router
dev.ip_wireless_port = 54321;

dev.walk='NewNewNewWalk'; --Walk with generalized walkkick definitions
dev.kick='NewNewKick'; --Extended kick that supports upper body motion

dev.walk='NewNewNewNewNewWalk'; 
	--Walk with upper body motion playback


--Sit/stand stance parameters
stance={};
stance.bodyHeightSit = 0.20;
stance.supportXSit = -0.010;
stance.bodyHeightDive= 0.295;
stance.bodyTiltDive = 0;
stance.bodyTiltStance=0*math.pi/180; --bodyInitial bodyTilt, 0 for webots
stance.dpLimitStance=vector.new({.04, .03, .07, .4, .4, .4});
stance.dpLimitSit=vector.new({.1,.01,.06,.1,.3,.1});

stance.dpLimitStance=vector.new({.04, .03, .07, .4, .9, .4});
stance.dpLimitDive = vector.new({.04, .03, .07, .4, .9, .4});


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

--FSM and behavior settings
fsm = {};
--SJ: loading FSM config  kills the variable fsm, so should be called first
loadconfig('FSM/Config_WebotsOP_FSM')
fsm.game = 'RoboCup';
fsm.head = {'GeneralPlayer'};
fsm.body = {'GeneralPlayer'};

--Behavior flags, should be defined in FSM Configs but can be overridden here
fsm.playMode = 3; --1 for demo, 2 for orbit, 3 for direct approach
fsm.enable_obstacle_detection = 1;
fsm.wait_kickoff = 1;
fsm.enable_walkkick = 1;
fsm.enable_sidekick = 1;
fsm.enable_dribble = 1;

fsm.daPost_check = 1;
fsm.daPostmargin = 15*math.pi/180;
fsm.variable_dapost = 1;


--FAST APPROACH TEST
--fsm.fast_approach = 1;
--fsm.bodyApproach.maxStep = 0.06;
fsm.fast_approach = 0;
fsm.bodyApproach.maxStep = 0.04;

fsm.enable_evade = 0;
--fsm.enable_evade = 1;--Randomly do evade kick
fsm.enable_evade = 2;--Do evade kick when obstructed


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

--Enable this for double pass
--[[
fsm.body={'DoublePassChallenge'};
dev.team='TeamDoublePass';
--]]

-- Team Parameters
team = {};
team.msgTimeout = 5.0;
team.tKickOffWear = 15.0;

team.walkSpeed = 0.25; --Average walking speed 
team.turnSpeed = 2.0; --Average turning time for 360 deg
team.ballLostPenalty = 4.0; --ETA penalty per ball loss time
team.fallDownPenalty = 4.0; --ETA penalty per ball loss time
team.nonAttackerPenalty = 0.8; -- dist from ball
team.nonDefenderPenalty = 0.5; -- dist from goal



--if ball is away than this from our goal, go support
team.support_dist = 3.0; 
team.supportPenalty = 0.5; --dist from goal

team.force_defender = 0; --Enable this to force defender

team.use_team_ball = 1;
team.team_ball_timeout = 3.0;  --use team ball info after this delay
team.team_ball_threshold = 0.5;

team.avoid_own_team = 1;
team.avoid_other_team = 1;




-- keyframe files
km = {};
km.standup_front = 'km_NSLOP_StandupFromFront.lua';
km.standup_back = 'km_NSLOP_StandupFromBack.lua';
km.standup_back2 = 'km_NSLOP_StandupFromBack3.lua';


--km.standup_back = 'km_NSLOP_StandupFromBack3.lua';
--km.kick_right = 'km_NSLOP_taunt1.lua';
--km.kick_left = 'km_NSLOP_StandupFromFront2.lua';

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


-- Low battery level
bat_med = 122; -- Slow down if voltage drops below 12.2V 
bat_low = 118; -- 11.8V warning

--Fall check
fallAngle = 40*math.pi/180;
falling_timeout = 0.3;

--Slow down top speed
--[[
fsm.bodyPosition.maxStep1 = 0.06;
fsm.bodyPosition.maxStep2 = 0.07;
fsm.bodyPosition.maxStep3 = 0.08;
--]]

--Shutdown Vision and use ground truth gps info only
use_gps_only = 0;
--use_gps_only = 1;

fsm.enable_walkkick = 0;
--fsm.enable_sidekick = 0;

--New multi-blob landmark detection code
vision.use_multi_landmark = 1;

-- obstacle avoidance challenge
obs_challenge = 1;
obs_challenge = 0;
fsm.enable_sidekick = 1;
fsm.thSideKick1 = 30*math.pi/180;
fsm.thSideKick2 = 135*math.pi/180;
fsm.thDistSideKick = 1.0;

use_rollback_getup = 1;
batt_max = 120; --only do rollback getup when battery is enough


--For doublepass
fsm.headTrack.timeout = 2.0 * speedFactor;
fsm.headTrack.tLost = 1.5 * speedFactor;
fsm.headTrack.minDist = 0.15; --Default value 0.30,If ball is closer than this, don't look up

--Roll backup setup
