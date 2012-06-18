module(..., package.seeall);

require('vector')
require('unix')

platform = {}; 
platform.name = 'OP'

function loadconfig(configName)
  local localConfig=require(configName);
  for k,v in pairs(localConfig) do
    Config[k]=localConfig[k];
  end
end

--Robot CFG should be loaded first to set PID values
local robotName=unix.gethostname();
has_claw = 0;
if (robotName=='sally') then has_claw = 1;end

if has_claw>0 then
--  loadconfig('Robot/Config_OPGripper_Robot') 
  loadconfig('Robot/Config_OPSally_Robot') 
  loadconfig('Walk/Config_OP_Walk')
--  walk.qLArm=math.pi/180*vector.new({90,20,-40});
--  walk.qRArm=math.pi/180*vector.new({90,-20,-40});
else
  loadconfig('Robot/Config_OP_Robot') 
  loadconfig('Walk/Config_OP_Walk')
end

loadconfig('World/Config_OP_World')
--loadconfig('Kick/Config_OP_Kick')
loadconfig('Kick/Config_OP_Kick_Slow')
--loadconfig('Kick/Config_OP_Kick3')
loadconfig('Vision/Config_OP_Vision')

--Location Specific Camera Parameters--
--loadconfig('Vision/Config_OP_Camera_VT')
--loadconfig('Vision/Config_OP_Camera_L512')
--loadconfig('Vision/Config_OP_Camera_L512_Day')
--loadconfig('Vision/Config_OP_Camera_RC12')
loadconfig('Vision/Config_OP_Camera_Ob_F1')

--loadconfig('Vision/Config_OP_Camera_Grasp')

-- Device Interface Libraries
dev = {};
dev.body = 'OPBody'; 
dev.camera = 'OPCam';
dev.kinematics = 'OPKinematics';
dev.ip_wired = '192.168.123.255';
dev.ip_wireless = '192.168.1.255';
dev.game_control='OPGameControl';
dev.team='TeamNSL';
--dev.walk='NewNewNewWalk';
dev.walk='NewNewNewNewNewWalk';
dev.kick = 'NewNewKick'
dev.gender = 1; -- 1 for body and 0 for girl 

speak = {}
speak.enable = false; 

-- Game Parameters
game = {};
game.teamNumber = 18;
--game.teamNumber = 26;

--Default role: 0 for goalie, 1 for attacker, 2 for defender

dev.gender = 1;
game.role = 1; --Default attacker

if (robotName=='scarface') then
  game.playerID = 1; 
elseif (robotName=='linus') then
  game.playerID = 2; 
elseif (robotName=='betty') then
  game.playerID = 3; 
elseif (robotName=='lucy') then
  game.playerID = 4; 
elseif (robotName=='felix') then
  game.playerID = 3; 
elseif (robotName=='jiminy') then
  game.playerID = 4; 

elseif (robotName=='hokie') then
  game.playerID = 5; 
  game.role = 0; --Default goalie
elseif (robotName=='sally') then
  game.playerID = 5; 
  game.role = 0; --Default goalie
end


game.role = 1;--hack

--Default team: 0 for blue, 1 for red  
game.teamColor = 0; --Blue team
--game.teamColor = 1; --Red team
game.robotName = robotName;
game.robotID = game.playerID;
game.nPlayers = 5;
--------------------

--FSM and behavior settings
fsm = {};
--SJ: loading FSM config  kills the variable fsm, so should be called first
loadconfig('FSM/Config_OP_FSM')
fsm.game = 'RoboCup';
fsm.head = {'GeneralPlayer'};
fsm.body = {'GeneralPlayer'};

--Behavior flags, should be defined in FSM Configs but can be overrided here
fsm.enable_obstacle_detection = 1;
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

team.avoid_own_team = 0;
team.avoid_other_team = 0;



-- keyframe files
km = {};
km.standup_front = 'km_NSLOP_StandupFromFront.lua';
km.standup_back = 'km_NSLOP_StandupFromBack.lua';
--km.standup_back = 'km_NSLOP_StandupFromBack3.lua';

if (robotName=='hokie') then
--  km.standup_back = 'km_NSLOP_StandupFromBackHokie.lua';
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
--fsm.goalie_reposition=0; --No reposition
fsm.goalie_reposition=1; --Yaw reposition
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


--Slow down maximum speed (for testing)
fsm.bodyPosition.maxStep1 = 0.05;
fsm.bodyPosition.maxStep2 = 0.06;
fsm.bodyPosition.maxStep3 = 0.06;

