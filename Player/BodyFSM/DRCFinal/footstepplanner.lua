local footstepplanner={}
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'
require'wcm'
require'hcm'

local sformat = string.format
local footY = Config.walk.footY

--2D surfaces
--x1 x2 y1 y2  z_center pitch
local surfaces2D={ 
  {0,0.40,     -0.2,0.2,          0.0,0}, 
  {0.40,0.80,  -0.2,0.2,  0.15,0},  
  {0.80,1.20,  -0.2,0.2,  0.30,0},  
  {1.20,1.60,  -0.2,0.2,  0.45,0}, 
  {1.60,2.00,  -0.2,0.2,  0.60,0}, 
  {2.00,2.40,  -0.2,0.2,  0.45,0},  
  {2.40,2.80,  -0.2,0.2,  0.30,0},  
  {2.80,3.20,  -0.2,0.2,  0.15,0},  
  {3.20,3.60,  -0.2,0.2,  0.0,0},  

  {0,0.40,     -0.6,-0.2,          0.0,0}, 
  {0.40,0.80,  -0.6,-0.2,  0,0},  
  {0.80,1.20,  -0.6,-0.2,  0.15,0},  
  {1.20,1.60,  -0.6,-0.2,  0.30,0}, 
  {1.60,2.00,  -0.6,-0.2,  0.45,0}, 
  {2.00,2.40,  -0.6,-0.2,  0.60,0},  
  {2.40,2.80,  -0.6,-0.2,  0.45,0},  
  {2.80,3.20,  -0.6,-0.2,  0.30,0},  
  {3.20,3.60,  -0.6,-0.2,  0.15,0},  
}


--This is for the lamp

  --x1 x2 y1 y2,       z_center, pitch

local surfaces2D={
    {0.0,0.30,    -1.2,1.2,           0,   0}, 
    {0.30,0.90,   -1.2,1.2,          0.08045,   -0.262}, 
    {0.90,1.50,   -1.2,1.2,          0.08045,    0.262}, 
    {1.50,2.00,    -1.2,1.2,           0,   0}, 
}

--[[

--Lamp-stair compound
local surfaces2D={
    {0.0,0.30,    -1.2,1.2,           0,   0}, 
    {0.30,0.90,   -1.2,1.2,          0.08045,   -0.262}, 
    {0.90,1.50,   -1.2,1.2,          0.08045,    0.262}, 
    {1.50,2.40,    -1.2,1.2,           0,   0}, 

 
  {2.40,2.80,  -0.2,0.2,  0.15,0},  
  {2.80,3.20,  -0.2,0.2,  0.30,0},  
  {3.20,3.60,  -0.2,0.2,  0.45,0}, 
  {3.60,4.00,  -0.2,0.2,  0.60,0}, 
  {4.00,4.40,  -0.2,0.2,  0.45,0},  
  {4.40,4.80,  -0.2,0.2,  0.30,0},  
  {4.80,5.20,  -0.2,0.2,  0.15,0},  
  {5.20,5.60,  -0.2,0.2,  0.0,0},  

}


--]]


--local toeX = 0.13
--local heelX = 0.11

--2cm margin
local toeX = 0.15
local heelX = -0.11

local kneeCheckX = 0.30

local heelLandX = -0.15 
local feetY = 0.08 -- is it correct?


local max_stride_x = 0.28
local stepheight_max = 0.16
local max_stride_land_x = 0.31



local function check_side(v1,v2,v)
  local vector1={v[1]-v1[1],v[2]-v1[2]}
  local vector2={v2[1]-v1[1],v2[2]-v1[2]}
  --cross product of two vectors
  return (vector1[1]*vector2[2]-vector1[2]*vector2[1])
end

local function check_inside(v1,v2,v3,v4, v)
  if check_side(v1,v2,v)>0 and
    check_side(v2,v3,v)>0 and
    check_side(v3,v4,v)>0 and
    check_side(v4,v1,v)>0 then    return true
  else return false
  end
end

local function check_inside_surface2D(s, v)
  return check_inside({s[1],s[4]},{s[2],s[4]},{s[2],s[3]},{s[1],s[3]},v)
end

local function check_inside_surface2D_margin(s,uFoot,toe_margin,heel_margin)
  local tL = util.pose_global({toe_margin,feetY,0},uFoot)
  local tR = util.pose_global({toe_margin,-feetY,0},uFoot)
  local hL = util.pose_global({heel_margin,feetY,0},uFoot)
  local hR = util.pose_global({heel_margin,-feetY,0},uFoot)  
  local check = (check_inside_surface2D(s,tL) and
          check_inside_surface2D(s,tR) and
          check_inside_surface2D(s,hL) and
          check_inside_surface2D(s,hR) )
  return check
end



local function find_current_surface_height(uFoot)
  for i=1,#surfaces2D do
    if check_inside_surface2D(surfaces2D[i],uFoot) then 
      local x_mid = (surfaces2D[i][1]+surfaces2D[i][2])/2
      local y_mid = (surfaces2D[i][3]+surfaces2D[i][4])/2
      local pitch = surfaces2D[i][6]  
      local surface_height = surfaces2D[i][5] - (uFoot[1]-x_mid) * math.tan(pitch)

      return surface_height,pitch
    end
  end
  return 0,0
end

local function is_reachable(uFoot,swing_z,support_z,support_pitch,uFootSupport)
  for i=1,#surfaces2D do
    

    local x_mid = (surfaces2D[i][1]+surfaces2D[i][2])/2
    local y_mid = (surfaces2D[i][3]+surfaces2D[i][4])/2
    local pitch = surfaces2D[i][6]


    local y_width = surfaces2D[i][4]-surfaces2D[i][3]
    local surface_height = surfaces2D[i][5] - (uFoot[1]-x_mid) * math.tan(pitch)
    

    local stepheight = surface_height-swing_z
    local is_first_climbing = surface_height-support_z
    --if the step is first step over inclined surface, it requires more toe room to prevent shin strike
    local toemargin, heelmargin = toeX, heelX

   -- if is_first_climbing>0 then toemargin = toeClimbX end
    if stepheight<0 and math.abs(support_pitch)<math.pi/180 then heelmargin = heelLandX end

    if math.abs(stepheight)<stepheight_max and math.abs(is_first_climbing)<stepheight_max and
      check_inside_surface2D_margin(surfaces2D[i],uFoot,toemargin, heelmargin) then 
      return stepheight,pitch 
    end
  end 
  return nil
end

local function sample_landing_positions(uFoot, swing_z, support_z,support_pitch,leftsupport)
  local possible_foot_positions={}
  local dir = hcm.get_step_dir()
  local s,e=0,0.32
  --
  if math.abs(support_pitch)*5*math.pi/180 then
    s,e=0,0.28
  end
  --

  if dir<0 then s,e = -0.32,0 end


  


  for j=0,0.04,0.01 do
    for i=s,e,0.01 do    
      local uFoot2 = util.pose_global({i,-j*leftsupport,0},uFoot)
      uFoot2[3]=0

      --Hack to track the center line
--      uFoot2[2] = -leftsupport*feetY



      local stepheight,pitch = is_reachable(uFoot2,swing_z,support_z,support_pitch,uFoot)
      if stepheight then table.insert(possible_foot_positions,{uFoot2, {stepheight,pitch,0}} )end
    end
    if #possible_foot_positions>0 then return possible_foot_positions end
  end
  return possible_foot_positions
end

local function select_best_landing_position(uFoot,candidate)
  local best_foot, best_zpr = {0,0,0}, {0,0,0}
  local no_solution = true

  local dir = hcm.get_step_dir()

  for i=1,#candidate do
    print(sformat("Step candidate: %.2f / z %.3f p %d",candidate[i][1][1], candidate[i][2][1], 
      candidate[i][2][2]*180/math.pi
      ))

    if dir>=0 then
      if candidate[i][1][1]>best_foot[1] or no_solution then
        no_solution = false
        best_foot = candidate[i][1]
        best_zpr = candidate[i][2]
      end
    else
      if candidate[i][1][1]<best_foot[1] or no_solution then
        no_solution = false
        best_foot = candidate[i][1]
        best_zpr = candidate[i][2]
      end
    end
  end
  if no_solution then
    print("No landing possition possible!")
    print("No landing possition possible!")
    print("No landing possition possible!")
    print("No landing possition possible!")
    print("No landing possition possible!")
    print("No landing possition possible!")

    hcm.set_step_nosolution(1)
    hcm.set_step_dir(0)
    return    
  end
  print(sformat("Landing position selected: %.2f / %.2f p %d", best_foot[1], best_zpr[1],best_zpr[2]*180/math.pi ))
  hcm.set_step_relpos(util.pose_relative(best_foot,uFoot))
  hcm.set_step_zpr(best_zpr)
end


function footstepplanner.getnextfoot(uFootSwing, uFootSupport,leftsupport)
  local possible_foot_positions,uFootSwing0={},nil
  local current_swing_z,current_swing_pitch = find_current_surface_height(uFootSwing)
  local current_support_z,current_support_pitch = find_current_surface_height(uFootSupport)


  local uFootSwingSupport = util.pose_relative(uFootSwing, uFootSupport)
  if uFootSwingSupport[2]>0 then uFootSwing0 = util.pose_global({0,2*footY,0},uFootSupport)--right support
  else uFootSwing0 = util.pose_global({0,-2*footY,0},uFootSupport) end

  local possible_foot_positions = 
    sample_landing_positions(uFootSwing0,current_swing_z,current_support_z,current_support_pitch, leftsupport)
  select_best_landing_position(uFootSwing,possible_foot_positions)
end


function footstepplanner.getnextstep()

--[[
  print("side:",check_side({0,0},{1,0},{0.5,1}))
  print("side:",check_side({0,0},{1,0},{0.5,0}))
  print("side:",check_side({0,0},{1,0},{0.5,-1}))

  print("inside:",check_inside({0,0},{1,0},{1,-1},{0,-1}, {0.5,-0.5}))
  print("inside:",check_inside({0,0},{1,0},{1,-1},{0,-1}, {0.5,0.5}))

--]]

  local pose = wcm.get_robot_pose()
  local uTorso = mcm.get_status_uTorso(uTorso)
  local uLeft = mcm.get_status_uLeft(uLeft)
  local uRight = mcm.get_status_uRight(uRight)

  local uLeftTorso = util.pose_relative(uLeft,uTorso)
  local uRightTorso = util.pose_relative(uRight,uTorso)

  local uLeftGlobal = util.pose_global(uLeftTorso,pose)
  local uRightGlobal = util.pose_global(uRightTorso,pose)
    

print(sformat("Current Torso:%.2f Left:%.2f Right:%.2f",uTorso[1],uLeft[1],uRight[1]))
print(sformat("Current pose:%.2f Left:%.2f Right:%.2f ",pose[1],uLeftGlobal[1],uRightGlobal[1]))
  
  local supportLeg = 0 --default right support
  if hcm.get_step_dir()>0 then --forward 
    if uLeftTorso[1]>uRightTorso[1] then supportLeg=1 end --left support
  elseif hcm.get_step_dir()<0 then --backward
    if uLeftTorso[1]<uRightTorso[1] then supportLeg=1 end
  end--left foot is leading foot

  if supportLeg==0 then
    footstepplanner.getnextfoot(uLeftGlobal,uRightGlobal,-1)
  else
    footstepplanner.getnextfoot(uRightGlobal,uLeftGlobal,1)
  end

  return supportLeg
end

return footstepplanner
