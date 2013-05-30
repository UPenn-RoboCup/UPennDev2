module(..., package.seeall);
local unix = require 'unix'
require('vector')
require('parse_hostname')

--Robot CFG should be loaded first to set PID values
local robotName=unix.gethostname();

platform = {};
platform.name = 'NaoV4'

listen_monitor=1
-- Game Parameters
-- init game table first since fsm need it
game = {};

-- Parameters Files
params = {}
params.name = {"Walk", "World", "Kick", "Vision", "FSM", "Camera","Robot"};

---Location Specific Camera Parameters--
params.Camera = "GraspChris"
params.Walk = "SlowSteady"
params.World = "SPL13Grasp"

util.LoadConfig(params, platform)

game.teamNumber = 25;
game.robotName = robotName;
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
dev.ip_wired = '192.168.123.255';
dev.ip_wired_port = 111111;
dev.ip_wireless = '192.168.1.255';
dev.ip_wireless_port = 54321
dev.game_control = 'NaoGameControl';
--dev.team='TeamSPL';
dev.team='TeamGeneral';
dev.walk = 'CleanWalk';
dev.kick = 'Walk/BasicKick';

--Speak enable
speakenable = 1;

-- FSM Parameters
fsm.game = 'RoboCup';
fsm.body = {'GeneralPlayer'};
fsm.head = {'GeneralPlayer'};

-- Team Parameters
--[[
team = {};
team.msgTimeout = 5.0;
team.nonAttackerPenalty = 6.0; -- eta sec
team.nonDefenderPenalty = 0.5; -- dist from goal
team.twoDefenders = 0;
--]]

--NEW Team parameters for TeamGeneral
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
team.test_teamplay = 0; --Enable this to immobilize attacker to test team beha$

--if ball is away than this from our goal, go support
team.support_dist = 3.0; 
team.supportPenalty = 0.5; --dist from goal
team.use_team_ball = 1;
team.team_ball_timeout = 3.0;  --use team ball info after this delay
team.team_ball_threshold = 0.5;
team.avoid_own_team = 1;
team.avoid_other_team = 1;

-- keyframe files
km = {};
km.standup_front = 'km_NaoV4_StandupFromFront.lua';
km.standup_back = 'km_NaoV4_StandupFromBack.lua';
km.time_to_stand = 30; -- average time it takes to stand up in seconds

--Goalie behavior parameters
goalie_dive = 1; --1 for arm only, 2 for actual diving
goalie_dive_waittime = 6.0; --How long does goalie lie down?
fsm.goalie_type = 3;--moving/move+stop/stop+dive/stop+dive+move
fsm.goalie_reposition=1; --No reposition / Yaw reposition / Position reposition
fsm.bodyAnticipate.thFar = {0.4,0.4,30*math.pi/180};
fsm.goalie_use_walkkick = 1;--should goalie use walkkick or long kick?

--Diving detection parameters
fsm.bodyAnticipate.timeout = 3.0;
fsm.bodyAnticipate.center_dive_threshold_y = 0.05; 
fsm.bodyAnticipate.dive_threshold_y = 1.0;
fsm.bodyAnticipate.ball_velocity_th = 1.0; --min velocity for diving
fsm.bodyAnticipate.ball_velocity_thx = -1.0; --min x velocity for diving
fsm.bodyAnticipate.rCloseDive = 2.0; --ball distance threshold for diving

--vision.ball.max_distance = 2.5; --temporary fix for GRASP lab
vision.ball.fieldsize_factor = 1.2; --check whether the ball is inside the field
vision.ball.max_distance = 2; --if ball is this close, just pass the test

