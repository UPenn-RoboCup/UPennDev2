module(..., package.seeall);
require('util')
require('vector')
require('parse_hostname')

platform = {}; 
platform.name = 'OP'

-- Parameters Files
params = {}
params.name = {"Robot", "Walk", "World", "Kick", "Vision", "FSM", "Camera"};
params.Walk = "Basic"
params.World = "Expo"
--Location Specific Camera Parameters--
params.Camera = "Grasp"

util.LoadConfig(params, platform)

-- Device Interface Libraries
dev = {};
dev.body = 'OPBody'; 
dev.camera = 'OPCam';
dev.kinematics = 'OPKinematics';
dev.ip_wired = '192.168.123.255';
dev.ip_wireless = '192.168.1.255';
dev.ip_wireless_port = 54321;
dev.game_control='OPGameControl';
dev.team='TeamNSL';
dev.walk='BasicWalk';
dev.kick = 'NewKick'

-- Game Parameters

game = {};
game.teamNumber = 18;
game.playerID = parse_hostname.get_player_id();
game.robotID = game.playerID;
game.teamColor = parse_hostname.get_team_color();
game.nPlayers = 5;
--------------------

--Default role is based on player ID
--1 for goalie, 2 for attacker, 3 for defender
game.role = game.playerID-1; --default attacker

--FSM and behavior settings
fsm.game = 'RoboCup';
fsm.head = {'GeneralPlayer'};
fsm.body = {'SimplePlayer'};

--Behavior flags, should be defined in FSM Configs but can be overrided here
fsm.enable_obstacle_detection = 1;
fsm.kickoff_wait_enable = 0;
fsm.playMode = 2; --1 for demo, 2 for orbit, 3 for direct approach
fsm.enable_walkkick = 0;
fsm.enable_sidekick = 0;

-- Team Parameters
team = {};
team.msgTimeout = 5.0;
team.nonAttackerPenalty = 6.0; -- eta sec
team.nonDefenderPenalty = 0.5; -- dist from goal

-- keyframe files
km = {};
km.standup_front = 'km_OP_StandupFromFront_slow.lua';
km.standup_back = 'km_OP_StandupFromBack_slow.lua';

-- Low battery level
-- Need to implement this api better...
bat_low = 100; -- 10V warning

