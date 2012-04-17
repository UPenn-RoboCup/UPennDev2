module(..., package.seeall);

require('Body')
require('walk')
require('util')
require('vector')
require('Config')
require('wcm')
require('gcm')
require('Team')

t0 = 0;

tasklist={
  --left player 
  {
    {1,{1,1.5,-math.pi/2}}, --approach the ball and kick to the position  
    {2,{0,-1.5,math.pi/2}}, --move to the pose and wait there
    {1,{-1.2,1.0,-math.pi/2}}, --approach the ball and kick to the position
    {2,{-2.5,-1,0}}, --move to the pose and wait there
    {1,{-3,0,0}}, --Kick to score
  },
  --right player
  {
    {2,{1,1.5,-math.pi/2}}, --move to the pose and wait tere
    {1,{0,-1,math.pi/2}}, --approach and kick
    {2,{-1.2,1.5,-math.pi/2}}, --move and wait
    {1,{-2.5,-1,0}}, --approach and kick
    {0,{-3,0,0}}, --do nothing
  },
}  

function entry()
  print(_NAME.." entry");
  t0 = Body.get_time();

  --Set target pose for current task
  role=gcm.get_team_role();
  team_task_state=gcm.get_team_task_state();

print(string.format("Role %d Task state %d %d",role,unpack(team_task_state)))
end

function update()
  local t = Body.get_time();
  team_task_state=gcm.get_team_task_state();
  role=gcm.get_team_role();
  if team_task_state[role]>team_task_state[3-role] then
    --wait for the other robot to keep up
    return;
  end  

  task_state=team_task_state[role]; 
  -- index start with 0 and is always a even number 
  current_task=tasklist[role][task_state/2+1];
  gcm.set_team_target(current_task[2]);

  if task_state/2+1 <=#tasklist[role] then
    --dispatch task
    if current_task[1]==1 then
      print("Passing started")
      team_task_state[role]=team_task_state[role]+1;
      return "pass";
    else
      print("Moving started")
      team_task_state[role]=team_task_state[role]+2;
      return "move";
    end
  else
    Motion.event("sit");
  end
end

function exit()
  gcm.set_team_task_state(team_task_state);
  print(string.format("Role %d Task state %d %d",role,unpack(team_task_state)))
end

