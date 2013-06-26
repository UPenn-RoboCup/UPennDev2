module(..., package.seeall);
require('util')
require('vector')
require('unix')

--Robot CFG should be loaded first to set PID values
local robotName=unix.gethostname();

platform = {}; 
platform.name = 'OP'

-- Parameters Files
params = {}
params.name = {"Robot", "Walk", "World", "Kick", "Vision", "FSM", "Camera"};
params.Kick = "Slow"

---Location Specific Camera Parameters--
--params.Camera = "Grasp"
--params.Camera = "Grasp_lowE_pink"
--params.Camera = "Eindhoven_lowC"
params.Camera = "Eindhoven_B"
--params.Camera = "Eindhoven_highC"
--params.Camera = "RC_13_25"

util.LoadConfig(params, platform)

-- Device Interface Libraries
dev = {};
dev.body = 'OPBody'; 
dev.camera = 'OPCam';
dev.kinematics = 'OPKinematics';
dev.ip_wired = '192.168.123.255';
dev.ip_wired_port = 111111;
dev.ip_wireless = '192.168.119.255'; --Our Router
--dev.ip_wireless = '192.168..255'; --Our Router
dev.ip_wireless_port = 54321;
dev.game_control='OPGameControl';
--dev.team='TeamNSL';
dev.team='TeamGeneral';
dev.walk='AwesomeWalk'
--dev.walk='CleanWalk'
dev.kick = 'PunchKick'
dev.gender = 1; -- 1 for boy and 0 for girl 
largestep_enable = true;
dev.largestep = 'ZMPStepKick';

speak = {}
speak.enable = false; 

-- Game Parameters
game = {};
game.teamNumber = 19;   --17 at RC12

--Default role: 0 for goalie, 1 for attacker, 2 for defender
ball_shift={0,0};
game.playerID = 1;
game.role = 1;  --Default role: attacker

if (robotName=='scarface') then
  game.playerID = 1; 
elseif (robotName=='linus') then
  game.playerID = 2; 
elseif (robotName=='betty') then
  game.playerID = 3; 
elseif (robotName=='lucy') then
  game.playerID = 4; 


elseif (robotName=='andy') then
  game.playerID = 1; 
elseif (robotName=='annie') then
  game.playerID = 2; 
elseif (robotName=='jiminy') then
  game.playerID = 3; 
end

--Default team: 0 for blue, 1 for red  
--game.teamColor = 0; --Blue team
game.teamColor = 1; --Red team
game.robotName = robotName;
game.robotID = game.playerID;
game.nPlayers = 5;
--------------------

--FSM and behavior settings
fsm.game = 'RoboCup';
fsm.head = {'GeneralPlayer'};
fsm.body = {'GeneralPlayer'};

-- Team Parameters
team = {};
team.msgTimeout = 5.0;
team.tKickOffWear =7.0;
team.walkSpeed = 0.25; --Average walking speed 
team.turnSpeed = 1.0; --Average turning time for 360 deg
team.ballLostPenalty = 4.0; --ETA penalty per ball loss time
team.fallDownPenalty = 4.0; --ETA penalty per ball loss time
team.nonAttackerPenalty = 0.8; -- distance penalty from ball
team.nonDefenderPenalty = 0.5; -- distance penalty from goal
team.force_defender = 0;--Enable this to force defender mode

--if ball is away than this from our goal, go support
team.support_dist = 3.0; 
team.supportPenalty = 0.5; --dist from goal

--Team ball parameters
team.use_team_ball = 0;
team.team_ball_timeout = 3.0;  --use team ball info after this delay
team.team_ball_threshold = 0.5; --Min score to use team ball
team.avoid_own_team = 1;
team.avoid_other_team = 0;
team.flip_correction = 0;
team.vision_send_interval = 10; --one labelB out of 10 team message


--FILP CORRECTION VARIABLES-------------------------
team.flip_correction = 1;
team.confused_threshold_x = 4.0;
team.confused_threshold_y = 4.0;
team.flip_threshold_x = 1.0;
team.flip_threshold_y = 1.5;
team.flip_check_t = 5.0; --keep confused for 5 sec

--team.confusion_handling = 0; --don't check for flipping
--team.confusion_handling = 1; --use CONFUSED role 
team.confusion_handling = 2; --keep the current role, move the ball to the side
-------------------------------------------------------------





-- keyframe files
km = {};
km.standup_front = 'km_NSLOP_StandupFromFront.lua';
km.standup_back = 'km_NSLOP_StandupFromBack.lua';
km.standup_back2 = 'km_NSLOP_StandupFromBack3.lua';

--Roll backup setup
use_rollback_getup = 1;

-- Low battery level
bat_low = 117; -- 11.7V warning for head LED (red blinking)
bat_med = 119; -- Slow down walking if voltage drops below this 
bat_led = {118,119,122,123,124,125}; --for back LED indicator

--Speak enable
speakenable = false;

--Fall check
fallAngle = 50*math.pi/180;
falling_timeout = 0.3;

--led_on = 0; --turn off eye led
led_on = 1; --turn on eye led

enable_ceremony = 0;
ceremony_score = 2; --Start ceremony if we are leading by this score

--green check turned off at this angle
vision.ball.th_headAngle = 10* math.pi/180;

--Let goalie log all the ball positions
fsm.goalie_type = 3;--moving/move+stop/stop+dive/stop+dive+move
goalie_dive = 1; --1 for arm only, 2 for actual diving
goalie_disable_arm = 1; 
goalie_log_balls = 1;
goalie_log_balls = 0;

vision.ball.max_distance = 2.0; --temporary fix for GRASP lab

listen_monitor = 1;
------------------------------------------------------------------------




------------------------------------------------------------------------
-- Demo match setting 
------------------------------------------------------------------------
--[[
led_on = 1; --turn on eye led
--Slow down maximum speed (for testing)
fsm.bodyPosition.maxStep1 = 0.04; 
fsm.bodyPosition.maxStep2 = 0.04;
fsm.bodyPosition.maxStep3 = 0.04;
--Disable walkkicks and sidekicks 
fsm.enable_walkkick = 0;  
fsm.enable_sidekick = 0;
-------------------------------------------------------------------------
--]]

-- avoider
--[[
fsm.head = {'ObstacleChallenge'}
fsm.body = {'ObstacleChallenge'}
fsm.avoidance_mode = 1 -- ball dribble
fsm.avoidance_mode = 0 -- walk towards goal, no ball 
fsm.avoidance_mode = 2 -- walk towards goal, no ball 
use_rollback_getup = 0;
-]]

use_kalman_velocity = 1;

fsm.goalie_reposition=3; --No reposition at all (for testing)
use_rollback_getup = 0;


--For THROW-IN---------------------------------------------------
--[[
walk.qLArm=math.pi/180*vector.new({90,25,-20});
walk.qRArm=math.pi/180*vector.new({90,-25,-20});
stance.qLArmSit = math.pi/180*vector.new({140,25,-40});
stance.qRArmSit = math.pi/180*vector.new({140,-25,-40});
use_rollback_getup = 0;
km.standup_front = 'km_NSLOP_StandupFromFront_Throw.lua';
km.standup_back = 'km_NSLOP_StandupFromBack_Throw.lua';
fsm.head = {'GeneralPlayer'};
fsm.body = {'ThrowinChallenge'};
--]]
-----------------------------------------------------------------

--[[
--For testing fast approach
fsm.bodyPosition.maxStep2 = 0.07;
fsm.bodyPosition.maxStep3 = 0.10;
walk.velLimitX={-.03,.10};
--]]

-------------------------------------------------------------
--FOR DEMO MATCH DAY 1

led_on = 1; --turn on eye led
--Slow down maximum speed (for testing)
fsm.bodyPosition.maxStep1 = 0.06; 
fsm.bodyPosition.maxStep2 = 0.06;
fsm.bodyPosition.maxStep3 = 0.06;
--Disable sidekick
fsm.enable_walkkick = 1;  
fsm.enable_sidekick = 0;

--Disable stepkick
dev.walk='CleanWalk';
largestep_enable = false;

--fsm.playMode = 2;--Orbit FSM
fsm.thDistStationaryKick = 2.0; --try do some stationary kick
-------------------------------------------------------------



------------------------------------------------------------------
-- FOR MATCH 1
led_on = 0; --turn on eye led
fsm.goalie_reposition=3; --No reposition at all (for testing)
