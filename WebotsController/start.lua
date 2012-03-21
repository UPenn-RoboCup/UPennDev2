require('controller');

print("\nStarting Webots Lua controller...");

playerID = os.getenv('PLAYER_ID') + 0;
teamID = os.getenv('TEAM_ID') + 0;


--SJ: ID-specific test code running 
--[[
if playerID == 101 then
  print("Starting test_walk");
  dofile("Player/Test/test_walk_webots.lua");
elseif playerID == 102 then
  print("Starting test_vision");
  dofile("Player/Test/test_vision_webots.lua");
else
  --Default: player
  print("Starting Player");
  dofile("Player/main.lua");
end
--]]

--  dofile("Player/Test/test_walk_webots.lua");
  dofile("Player/Test/test_vision_webots.lua");


--dofile("Player/Test/test_joints_webots.lua");
--dofile("Player/Test/test_punch_webots_op.lua");
--dofile("Player/Test/test_stretcher.lua")
