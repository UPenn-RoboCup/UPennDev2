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

function robocupplanner.getVelocity(pose,target_pose )

  -- Distance to the waypoint
  local homeRelative = util.pose_relative(target_pose,pose)
  local rHomeRelative = math.sqrt(homeRelative[1]^2+homeRelative[2]^2)
  local aHomeRelative = util.mod_angle(math.atan2(homeRelative[2],homeRelative[1]))
  local homeRot = math.abs(aHomeRelative)

  if math.abs(homeRelative[1])<0.4 and math.abs(homeRelative[2])<0.4 and math.abs(
  	util.mod_angle(pose[3]-target_pose[3]))<0.1 then
  print("reached")
  	return {0,0,0},true
  end

--  print(homeRelative[1],homeRelative[2],util.mod_angle(pose[3]-target_pose[3]))

  maxStep = 0.05
  if rHomeRelative>0.5 then maxStep = 0.10 end
  rTurn = 0.6
  local vx,vy,va = 0,0,0

  aTurn = math.exp(-0.5*(rHomeRelative/rTurn)^2)

  vx = maxStep * homeRelative[1]/rHomeRelative
  vy = maxStep * homeRelative[2]/rHomeRelative
  va = 0.5* (aTurn*homeRelative[3])
  		+(1-aTurn)*aHomeRelative

  return{vx,vy,va},false
end


return robocupplanner