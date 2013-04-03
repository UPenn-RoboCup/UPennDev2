-- Get Computer for Lib suffix
package.cpath = './?.so;' .. package.cpath;
require('controller');

cwd = os.getenv('PWD')
cwd = cwd ..'/Player'
playerID = os.getenv('PLAYER_ID') + 0;
teamID = os.getenv('TEAM_ID') + 0;

print("\nStarting Webots Lua controller...");
print("CWD:",cwd)
print("Team, Player:",teamID,playerID)

--SJ: Team-specific test code running 
if teamID == 98 then
  print("Starting test_walk");
  dofile("Player/Test/test_walk_webots.lua");
elseif teamID == 99 then
  print("Starting test_vision");
  dofile("Player/Test/test_vision_webots.lua");
elseif teamID==22 then
  print('laser')
  dofile("Player/Test/test_laser.lua");
elseif teamID==23 then
  print('arms_ik')
  dofile("Player/Test/test_arms.lua");
elseif teamID==24 then
  print('arms_fk')
  dofile("Player/Test/test_arms_fk.lua");
elseif teamID==25 then
  print('wheeltest')
  dofile("Player/Test/test_wheel.lua");
elseif teamID==26 then
  print('Ladder test')
  dofile("Player/Test/test_ladder.lua");
elseif teamID==27 then
  print('Ladder test')
  dofile("Player/Test/test_ladder_qual4.lua");
elseif teamID==28 then
  print('Door test')
  dofile("Player/Test/test_door.lua");
elseif teamID==29 then
  print('Arm only test')
  dofile("Player/Test/test_arm_only.lua");
elseif teamID==31 then
  print('Arm only test')
  dofile("Player/Test/test_step.lua");
else
	--Default
--  dofile("Player/Test/test_joints_webots.lua");
--  dofile("Player/Test/test_walk_webots.lua");
  dofile("Player/Test/test_vision_webots.lua");
--	dofile("Player/Test/test_main_webots.lua");
--  dofile("Player/main.lua");
--  dofile("Player/main.lua");
--  dofile("Player/Test/test_box.lua");
end

