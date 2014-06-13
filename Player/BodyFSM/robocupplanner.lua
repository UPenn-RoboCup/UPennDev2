local robocupplanner={}
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'

function robocupplanner.getTargetPose(pose,ballGlobal)


  local kickoffset = 0.5

  local goalL = {4.5,Config.world.goalWidth/2}
  local goalR = {4.5,-Config.world.goalWidth/2}
  local goal = {4.5,0}

  local goaldir = vector.new({goal[1]-ballGlobal[1],goal[2]-ballGlobal[2],0})
  local goaldist = math.sqrt(goaldir[1]*goaldir[1]+goaldir[2]*goaldir[2])

  local kickpos = ballGlobal - kickoffset * goaldir/goaldist
  kickpos[3] = math.atan2(goaldir[2],goaldir[1])
  return kickpos
end

return robocupplanner