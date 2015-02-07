local footstepplanner={}
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'
require'wcm'
require'hcm'

local sformat = string.format

--Temporary 1D surfaces
local surfaces={
  {-100,0.40,0.0,0},  -- x0 x1 z pitch
  {0.40,0.80,0.15,0},  -- x0 x1 z pitch
  {0.80,0.120,0.30,0},  -- x0 x1 z pitch
  {0.120,0.160,0.45,0},  -- x0 x1 z pitch
  {0.160,0.200,0.30,0},  -- x0 x1 z pitch
  {0.200,0.240,0.15,0},  -- x0 x1 z pitch
  {0.240,100,0,0},  -- x0 x1 z pitch
}


--local toeX = 0.13
--local heelX = 0.11

--2cm margin
local toeX = 0.15
local heelX = 0.13


local stride_max = 0.28
local stepheight_max = 0.16

function footstepplanner.getnextfoot(uFootSwing, uFootSupport)
  local possible_foot_positions={}
  local current_swingfoot_z = 0
  for i=1,#surfaces do
    if surfaces[i][2]-toeX>uFootSwing[1] and surfaces[i][1]+heelX>uFootSwing[1] then
      current_swingfoot_z = surfaces[i][3]
    end
  end

  print(sformat("Current foot height: %.2f",current_swingfoot_z))
  print(sformat("Swing foot X: %.2f",uFootSwing[1]))

  --enumerate all the possible foot transitions
  for i=1,#surfaces do
    local surface_max = surfaces[i][2] - toeX
    local surface_min = surfaces[i][1] + heelX

    local stride_max = uFootSupport[1]+stride_max
    local stride_min = uFootSupport[1]+stride_max

    local step_max = math.min(surface_max,stride_max)
    local step_min = math.max(surface_min,stride_min)

    local step_height = surfaces[i][3]-current_swingfoot_z

    if step_max>step_min and math.abs(current_swingfoot_z - surfaces[i][3])<stepheight_max then
      --Found a candidate for stepping
      table.insert(possible_foot_positions,{step_max,step_min,step_height})
      print(sformat("step candidate: Xmax %.2f Xmin %.2f Height %.2f",step_max,step_min,step_height))
    end
  end

--  hcm.

  step_relpos = hcm.get_step_relpos() 
  step_zpr = hcm.get_step_zpr()


end


function footstepplanner.getnextstep()
  local pose = wcm.get_robot_pose()
  local uTorso = mcm.get_status_uTorso(uTorso)
  local uLeft = mcm.get_status_uLeft(uLeft)
  local uRight = mcm.get_status_uRight(uRight)

  local uLeftTorso = util.pose_relative(uLeft,uTorso)
  local uRightTorso = util.pose_relative(uLeft,uTorso)

  local uLeftGlobal = util.pose_global(uLeftTorso,pose)
  local uRightGlobal = util.pose_global(uRightTorso,pose)
    
print(sformat("Current pose:%.2f Left:%.2f Right:%.2f ",pose[1],uLeftGlobal[1],uRightGlobal[1]))


  if uLeftTorso[1]>uRightTorso[1] then --left foot is leading foot
    footstepplanner.getnextfoot(uRightGlobal,uLeftGlobal)
  else
    footstepplanner.getnextfoot(uLeftGlobal,uRightGlobal)
  end
end

return footstepplanner
