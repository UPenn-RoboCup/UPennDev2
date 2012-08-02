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
if (robotName == 'sally') then 
--  params.Robot = "Gripper_Robot" 
  params.Robot = "Sally_Robot" 
end
params.Kick = "Slow"
--params.Kick = "3"

---Location Specific Camera Parameters--
--params.Camera = "VT"
params.Camera = "Grasp"
--params.Camera = "Grasp_obs"
--params.Camera = "L512"
--params.Camera = "L512_Day"

util.LoadConfig(params, platform)

-- Device Interface Libraries
dev = {};
dev.body = 'OPBody'; 
dev.camera = 'OPCam';
dev.kinematics = 'OPKinematics';
dev.ip_wired = '192.168.123.255';
dev.ip_wired_port = 111111;
dev.ip_wireless = '192.168.1.255'; --Our Router
dev.ip_wireless_port = 54321;
dev.game_control='OPGameControl';
dev.team='TeamNSL';
dev.walk='NewNewNewWalk' --For Grasp like surfaces, USED mostly!
--dev.walk='N5Walk';	 --For RC12 @ Mexico
dev.kick = 'NewNewKick'
dev.gender = 1; -- 1 for boy and 0 for girl 

speak = {}
speak.enable = false; 

-- Game Parameters
game = {};
game.teamNumber = 18;   --17 at RC12
--game.teamNumber = 26;

--Default role: 0 for goalie, 1 for attacker, 2 for defender

dev.gender = 1;
game.role = 1; --Default attacker

ball_shift={0,0};
game.playerID = 1;
if (robotName=='scarface') then
  game.playerID = 4; 
elseif (robotName=='linus') then
  game.playerID = 2; 
elseif (robotName=='betty') then
  game.playerID = 3; 
elseif (robotName=='lucy') then
  game.playerID = 1; 
elseif (robotName=='felix') then
  game.playerID = 2; 
elseif (robotName=='jiminy') then
  game.playerID = 5; 
elseif (robotName=='hokie') then
  game.playerID = 3; 
  game.role = 0; --Default goalie
elseif (robotName=='sally') then
  game.playerID = 5; 
  game.role = 0; --Default goalie
end

game.role = 1;--hack

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

--Behavior flags, should be defined in FSM Configs but can be overrided here
fsm.enable_obstacle_detection = 0;
fsm.kickoff_wait_enable = 0;
fsm.playMode = 3; --1 for demo, 2 for orbit, 3 for direct approach
fsm.forcePlayer = 0; --1 for attacker, 2 for defender, 3 for goalie 
fsm.enable_walkkick = 0; --Testing
fsm.enable_sidekick = 0;
fsm.daPost_check = 1; --aim to the side when close to the ball
fsm.daPostmargin = 15*math.pi/180;
fsm.variable_dapost = 1;


--FAST APPROACH TEST
fsm.fast_approach = 0;
--fsm.bodyApproach.maxStep = 0.06;

--1 for randomly doing evade kick
--2 for using obstacle information
--fsm.enable_evade = 0;
fsm.enable_evade = 0;

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
team.use_team_ball = 1;
team.team_ball_timeout = 3.0;  --use team ball info after this delay
team.team_ball_threshold = 0.5; --Min score to use team ball

team.avoid_own_team = 1;
team.avoid_other_team = 0;



-- keyframe files
km = {};
km.standup_front = 'km_NSLOP_StandupFromFront.lua';
km.standup_back = 'km_NSLOP_StandupFromBack.lua';
km.standup_back2 = 'km_NSLOP_StandupFromBack3.lua';


if (robotName=='sally') then 
  km.standup_front = 'km_NSLOP_StandupFromFrontSally.lua'; 
  km.standup_back = 'km_NSLOP_StandupFromBackSally.lua';
end

-- Low battery level
-- Need to implement this api better...
bat_low = 117; -- 11.7V warning
bat_med = 119; -- Slow down walking if voltage drops below this 

bat_led = {118,119,122,123,124,125}; --for back LED indicator

gps_only = 0;

goalie_dive = 1; --1 for arm only, 2 for actual diving
--goalie_dive = 2; --1 for arm only, 2 for actual diving
goalie_dive_waittime = 3.0; --How long does goalie lie down?
--fsm.goalie_type = 1;--moving/move+stop/stop+dive/stop+dive+move
--fsm.goalie_type = 2;--moving/move+stop/stop+dive/stop+dive+move
fsm.goalie_type = 3;--moving/move+stop/stop+dive/stop+dive+move
fsm.goalie_reposition=0; --No reposition
--fsm.goalie_reposition=1; --Yaw reposition
--fsm.goalie_reposition=2; --Position reposition

fsm.goalie_use_walkkick = 1; --should goalie use front walkkick?

--Goalie diving detection parameters
fsm.bodyAnticipate.timeout = 3.0;
fsm.bodyAnticipate.center_dive_threshold_y = 0.05; 
fsm.bodyAnticipate.dive_threshold_y = 1.0;
fsm.bodyAnticipate.ball_velocity_th = 1.0; --min velocity for diving
fsm.bodyAnticipate.ball_velocity_thx = -1.0; --min x velocity for diving
fsm.bodyAnticipate.rCloseDive = 2.0; --ball distance threshold for diving

--Speak enable
speakenable = false;

--Fall check
fallAngle = 50*math.pi/180;
falling_timeout = 0.3;

--led_on = 0; --turn off eye led
led_on = 1; --turn on eye led

--New multi-blob landmark detection code
vision.use_multi_landmark = 1;

------------------------------------------------------------------------
-- Demo setting 1

--led_on = 0; --turn on eye led

--Slow down maximum speed (for testing)

fsm.bodyPosition.maxStep1 = 0.04; --Default 0.06
fsm.bodyPosition.maxStep2 = 0.04;
fsm.bodyPosition.maxStep3 = 0.04;


--Disable walkkicks and sidekicks 
fsm.enable_walkkick = 0; --Testing 
fsm.enable_sidekick = 0;

--Disable diving
fsm.goalie_type = 3;--moving/move+stop/stop+dive/stop+dive+move
goalie_dive = 1; --1 for arm only, 2 for actual diving

--Let goalie log all the ball positions
goalie_disable_arm = 1; 
goalie_log_balls = 0;


--Slow down kick waiting time
--[[
fsm.bodyKick.tStartWait = 1.0;
fsm.bodyKick.tStartWaitMax = 1.2;
--]]

fsm.bodyKick.tStartWait = 0.6;
fsm.bodyKick.tStartWaitMax = 0.8;

--[[
--Power down walkkick
walk.walkKickDef["FrontLeft"]={
  {0.30, 1, 0, 0.035 , {0,0}, 0.6, {0.06,0,0} },
  {0.40, 2, 1, 0.05 , {0.02,-0.02}, 0.5, {0.06,0,0}, {0.07,0,0} },
  {walk.tStep, 1, 0, 0.035 , {0,0}, 0.5, {0.04,0,0} },
}
walk.walkKickDef["FrontRight"]={
  {0.30, 1, 1, 0.035 , {0,0}, 0.4, {0.06,0,0} },
  {0.40, 2, 0, 0.05 , {0.02,0.02}, 0.5,  {0.06,0,0}, {0.07,0,0} },
  {walk.tStep, 1, 1, 0.035 , {0,0}, 0.5, {0.04,0,0} },
}
--Close-range walkkick (step back and then walkkick)
walk.walkKickDef["FrontLeft2"]={
  {0.30, 1, 1, 0.035 , {0,0}, 0.4, {-0.06,0,0} },
  {0.30, 1, 0, 0.035 , {0.02,0}, 0.6, {0.06,0,0} },
  {0.40, 2, 1, 0.05 , {0.0,-0.02}, 0.5, {0.06,0,0}, {0.07,0,0} },
  {walk.tStep, 1, 0, 0.035 , {0,0}, 0.5, {0.04,0,0} },
}
walk.walkKickDef["FrontRight2"]={
  {0.30, 1, 0, 0.035 , {0,0}, 0.6, {-0.06,0,0} },
  {0.30, 1, 1, 0.035 , {0.02,0}, 0.4, {0.06,0,0} },
  {0.40, 2, 0, 0.05 , {0.0,0.02}, 0.5,  {0.06,0,0}, {0.07,0,0} },
  {walk.tStep, 1, 1, 0.035 , {0,0}, 0.5, {0.04,0,0} },
}
--]]

-------------------------------------------------------------------------


-----------------------------------------------------------------------
-- DEMO setting 2

--Fast kick starting time
fsm.bodyKick.tStartWait = 0.7;
fsm.bodyKick.tStartWaitMax = 0.9;

--Let goalie log balls
goalie_log_balls = 1;

fsm.goalie_type = 4;--moving/move+stop/stop+dive/stop+dive+move
goalie_dive = 1; --1 for arm only, 2 for actual diving

fsm.enable_walkkick = 1; --Enable front walkkick only


--Slow down maximum speed (for testing)
fsm.bodyPosition.maxStep1 = 0.04; --Default 0.06
fsm.bodyPosition.maxStep2 = 0.04;
fsm.bodyPosition.maxStep3 = 0.04;


-----------------------------------------------------------------------
--Setting for match #4

--led_on = 0; --turn off eye led
--led_on = 1; --turn ON eye led
fsm.enable_sidekick = 0;

-----------------------------------------------------------------------

--Sidekick testing
fsm.enable_sidekick = 0;
fsm.thSideKick1 = 30*math.pi/180;
fsm.thSideKick2 = 135*math.pi/180;
fsm.thDistSideKick = 1.0;

----------------------------------
-- obstacle avoidance challenge
obs_challenge = 0;
--Roll backup setup
use_rollback_getup = 1;

---------------------------------------------------------------
-- FOR SEMIFINAL
fsm.goalie_type = 2;--moving and stop goalie
fsm.goalie_reposition=1; --Yaw reposition
--[[
--maximum speed
fsm.bodyPosition.maxStep1 = 0.06;
fsm.bodyPosition.maxStep2 = 0.07;
fsm.bodyPosition.maxStep3 = 0.08;
--]]
bat_med = 119; -- Slow down walking if voltage drops below this 

fsm.daPostmargin = 20*math.pi/180; --More margin for kick to the side
fsm.bodyApproach.ballYMin = 0.16; --Tighter orbit radius

--green check turned off at this angle
vision.ball.th_headAngle = 10* math.pi/180;

world.postDiameter = 0.12;  --Thicker 
world.goalHeight = 0.80;
world.goalWidth = 1.40;

vision.goal.distanceFactorCyan = 1.15; 
vision.goal.distanceFactorYellow = 1.25; 
vision.landmark.distanceFactorCyan = 1.1; 
vision.landmark.distanceFactorYellow = 1.1; 


enable_ceremony = 0;
ceremony_score = 2;
-----------------------------------------------------------------
-- FINAL MATCH CONFIG

enable_ceremony = 1;
ceremony_score = 3; --3 goal difference
batt_max = 120; --12.0V rollback getup thershold
--If ball is closer than this don't look up
fsm.headTrack.minDist = 0.30;

------------------------------------------------------------------
-- Boxer
--[[
fsm.game = 'RoboCup';
fsm.head = {'GeneralPlayer'};
fsm.body = {'Boxer'};
dev.team = 'TeamBox'
dev.walk='B5Walk';
game.gcTimeout = 2;
team.msgTimeout = 1.0;
game.playerID = 1
use_rollback_getup = 0;
--]]
--
-----------------------------------------------------------------
-- avoider
fsm.head = {'ObstacleChallenge'}
fsm.body = {'ObstacleChallenge'}
fsm.avoidance_mode = 1 -- ball dribble
fsm.avoidance_mode = 0 -- walk towards goal, no ball 


