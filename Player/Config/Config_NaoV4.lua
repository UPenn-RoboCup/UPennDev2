module(..., package.seeall);

require('vector')
require('parse_hostname')

platform = {};
platform.name = 'Nao'

function loadconfig(configName)
  local localConfig=require(configName);
  for k,v in pairs(localConfig) do
    Config[k]=localConfig[k];
  end
end

loadconfig('Walk/Config_Nao_Walk')
loadconfig('World/Config_Nao_World')
loadconfig('Kick/Config_Nao_Kick')


loadconfig('Vision/Config_NaoV4_Vision')
--loadconfig('Vision/Config_Nao_Vision')

--Location Specific Camera Parameters--
loadconfig('Vision/Config_NaoV4_Camera')
--loadconfig('Vision/Config_Nao_Camera_blimp')





-- Devive Interface Libraries
dev = {};
dev.body = 'NaoBody'; 
dev.camera = 'NaoCam';
dev.kinematics = 'NaoKinematics';
dev.ip_wired = '192.168.0.255';
dev.ip_wireless = '192.168.1.255';
dev.game_control = 'NaoGameControl';
dev.team='TeamSPL';
dev.walk = 'NewNewNewWalk';
dev.kick = 'NewKick';

-- Game Parameters

game = {};
game.teamNumber = 26;
game.playerID = parse_hostname.get_player_id();
game.robotID = game.playerID;
game.teamColor = parse_hostname.get_team_color();
game.role = game.playerID-1; -- 0 for goalie
game.nPlayers = 4;


-- FSM Parameters
--[[
fsm = {};
fsm.game = 'RoboCup';
if (game.playerID == 1) then
  fsm.body = {'NaoGoalie'};
  fsm.head = {'NaoGoalie'};
else
  fsm.body = {'NaoPlayer'};
  fsm.head = {'NaoPlayer'};
end
--]]

--------------------------------------------------------------------
--GeneralPlayer FSM test
fsm = {};
loadconfig('FSM/Config_Nao_FSM')--For generalPlayer FSM
fsm.game = 'RoboCup';
fsm.body = {'GeneralPlayer'};
--fsm.head = {'GeneralPlayer'};
fsm.head = {'NaoPlayer'};

--Behavior flags, should be defined in FSM Configs but can be overrided here
fsm.enable_obstacle_detection = 1;
fsm.wait_kickoff = 0;
fsm.playMode = 2; --1 for demo, 2 for orbit, 3 for direct approach
fsm.enable_walkkick = 0;
fsm.enable_sidekick = 0;
-------------------------------------------------------------------

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
head.bodyTilt = 0;

-- keyframe files

km = {};
km.kick_right = 'km_Nao_KickForwardRight.lua';
km.kick_left = 'km_Nao_KickForwardLeft.lua';
km.standup_front = 'km_Nao_StandupFromFrontFaster.lua';
km.standup_back = 'km_Nao_StandupFromBackFasterNew.lua';

--Sit/stand stance parameters
stance={};
stance.bodyHeightSit = 0.18;
stance.supportXSit = 0.023;
stance.bodyHeightDive= 0.25;
stance.dpLimitSit=vector.new({.1,.01,.03,.1,.3,.1});
stance.bodyTiltStance=0*math.pi/180; --bodyInitial bodyTilt, 0 for webots
stance.dpLimitStance=vector.new({.04, .03, .04, .05, .4, .1});
stance.delay = 80; --amount of time to stand still after standing to regain balance.

--For compatibility with OP
--Should be more generally handled in Body..
servo={};
servo.pid=0;
