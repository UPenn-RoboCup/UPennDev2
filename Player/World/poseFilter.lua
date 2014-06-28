local poseFilter = {}

local vector = require'vector'
local util = require 'util'

--TODO: remove unnecessary stuff
local N = Config.world.nParticle
local xBoundary = Config.world.xBoundary
local yBoundary = Config.world.yBoundary
local xMax = Config.world.xMax
local yMax = Config.world.yMax

local goalWidth = Config.world.goalWidth
local goalUpper = Config.world.goalUpper
local goalLower = Config.world.goalLower
local postAll = {goalUpper[1], goalUpper[2], goalLower[1], goalLower[2]}
local postLeft = vector.new({goalUpper[1], goalLower[1]})
local postRight = vector.new({goalUpper[2], goalLower[2]})

local use_new_goalposts= Config.world.use_new_goalposts or 0 --Triangulation method selection
local rGoalFilter = Config.world.rGoalFilter
local aGoalFilter = Config.world.aGoalFilter
local rPostFilter = Config.world.rPostFilter
local aPostFilter = Config.world.aPostFilter
local rKnownGoalFilter = Config.world.rKnownGoalFilter or Config.world.rGoalFilter
local aKnownGoalFilter = Config.world.aKnownGoalFilter or Config.world.aGoalFilter
local rUnknownPostFilter = Config.world.rUnknownPostFilter or Config.world.rPostFilter
local aUnknownPostFilter = Config.world.aUnknownPostFilter or Config.world.aPostFilter

local xp = .5*xMax*vector.new(util.randn(N)) -- x coordinate of each particle
local yp = .5*yMax*vector.new(util.randn(N)) -- y coordinate
local ap = 2*math.pi*vector.new(util.randu(N)) -- angle
local wp = vector.zeros(N) -- weight

---Initializes a gaussian distribution of particles centered at p0
--@param p0 center of distribution
--@param dp scales how wide the distrubution is
function poseFilter.initialize(p0, dp)
  p0 = p0 or {0, 0, 0}
  dp = dp or {.1*xMax, .1*yMax, 5*DEG_TO_RAD}

  xp = p0[1]*vector.ones(N) + dp[1]*vector.new(util.randn(N))
  yp = p0[2]*vector.ones(N) + dp[2]*vector.new(util.randn(N))
  ap = p0[3]*vector.ones(N) + dp[3]*(vector.new(util.randu(N))-0.5*vector.ones(N))
  wp = vector.zeros(N)
end

---Returns best pose out of all particles
function poseFilter.get_pose()
  local wmax, imax = util.max(wp)
  return xp[imax], yp[imax], mod_angle(ap[imax])
end

local function calculate_pose_angle_old(pos,v,pose3,debug)
  --Calculate the closest target pose with same posea
  local a = math.atan2(v[2], v[1]) --We only use this! 
  local r = math.sqrt(v[2]^2+ v[1]^2)
  local ca = math.cos(a+pose3[3])
  local sa = math.sin(a+pose3[3])
  local x = pos[1] - r*ca
  local y = pos[2] - r*sa  
  return {x,y,pose3[3]},1
end

local function calculate_pose_angle(pos,v,pose3,debug)
  --Calculate the closest target pose with same posea
  local a = math.atan2(v[2], v[1]) --We only use this! 
  local ca = math.cos(a+pose3[3])
  local sa = math.sin(a+pose3[3])

  --Line from each particle
  --{x,y} + t*{cos(a + posea ), sin(a+posea)} = pos
  --Closest point to (posex,posey)
  --t = ca*(v[1]-posex) + sa*(v[2]-posey)

  local t_min = ca*(pos[1]-pose3[1])+sa*(pos[2]-pose3[2])
  local x_min = pos[1]-t_min*ca
  local y_min = pos[2]-t_min*sa
  
  if debug and Config.debug.goal_localization and t_min>0 then    
    print("Landmark pose:",pos[1],pos[2])
    print("Current pose:",pose3[1],pose3[2])
    print("Calculated pose:",x_min,y_min,pose3[3])
    print("t:",t_min)
  end

  return {x_min,y_min,pose3[3]},t_min
end

local function landmark_observation_angle(pos, v, rFilter, aFilter)
  --Update particles only using the landmark angle
  local r = math.sqrt(v[1]^2 + v[2]^2)
  local a = math.atan2(v[2], v[1]) --We only use this! 
  local rSigma = .15*r -- + 0.10
  local aSigma = 2*math.pi/180
  local rFilter = rFilter or 0.02
  local aFilter = aFilter or 0.04

  --Calculate best matching landmark pos to each particle
  local dxp = {}
  local dyp = {}
  local dap = {}

  for ip = 1,N do
    local dx = {}
    local dy = {}
    local dr = {}
    local da = {}
    local err = {}    
    for ipos = 1,#pos do
      local pose_target,t = calculate_pose_angle(pos[ipos],v,{xp[ip],yp[ip],ap[ip]},ip==1)
--      local pose_target,t = calculate_pose_angle_old(pos[ipos],v,{xp[ip],yp[ip],ap[ip]},ip==1)
--      local pose_target,t = calculate_pose_angle(pos[ipos],v,{xp[ip],yp[ip],ap[ip]})      
      if t>0 then
        dx[ipos] = pose_target[1] - xp[ip]
        dy[ipos] = pose_target[2] - yp[ip]
        dr[ipos] = math.sqrt(dx[ipos]^2 + dy[ipos]^2) - r      
        da[ipos] = mod_angle(math.atan2(dy[ipos],dx[ipos]) - (ap[ip] + a))
        err[ipos] = (dr[ipos]/rSigma)^2 + (da[ipos]/aSigma)^2
      else
        dx[ipos],dy[ipos],dr[ipos],da[ipos] = 0,0,0,0
        err[ipos] = math.huge
      end
    end
    local errMin, imin = util.min(err)

    --Update particle weights:
    wp[ip] = wp[ip] - errMin
    dxp[ip] = dx[imin]
    dyp[ip] = dy[imin]
    dap[ip] = da[imin]
  end

  --Filter toward best matching landmark position:  
  for ip = 1,N do
    xp[ip] = xp[ip] + rFilter * dxp[ip]
    yp[ip] = yp[ip] + rFilter * dyp[ip]
--    ap[ip] = ap[ip] + aFilter * dap[ip]
    -- check boundary
    xp[ip] = math.min(xMax, math.max(-xMax, xp[ip]))
    yp[ip] = math.min(yMax, math.max(-yMax, yp[ip]))
  end
end

---Updates particles with respect to the detection of a landmark
--@param pos Table of possible positions for a landmark
--@param v x and y coordinates of detected landmark relative to robot
--@param rLandmarkFilter How much to adjust particles according to
--distance to landmark
--@param aLandmarkFilter How much to adjust particles according to
--angle to landmark
local function landmark_observation(pos, v, rFilter, aFilter)

  if Config.use_angle_localization then
    return landmark_observation_angle(pos,v,rFilter,aFilter)
  end

  local r = math.sqrt(v[1]^2 + v[2]^2)
  local a = math.atan2(v[2], v[1])
  local rSigma = .15*r -- + 0.10
  local aSigma = 2*math.pi/180
  local rFilter = rFilter or 0.02
  local aFilter = aFilter or 0.04

  --Calculate best matching landmark pos to each particle
  local dxp = {}
  local dyp = {}
  local dap = {}
  for ip = 1,N do
    local dx = {}
    local dy = {}
    local dr = {}
    local da = {}
    local err = {}
    for ipos = 1,#pos do
      dx[ipos] = pos[ipos][1] - xp[ip]
      dy[ipos] = pos[ipos][2] - yp[ip]
      dr[ipos] = math.sqrt(dx[ipos]^2 + dy[ipos]^2) - r
      da[ipos] = mod_angle(math.atan2(dy[ipos],dx[ipos]) - (ap[ip] + a))
      err[ipos] = (dr[ipos]/rSigma)^2 + (da[ipos]/aSigma)^2
    end
    local errMin, imin = util.min(err)

    --Update particle weights:
    wp[ip] = wp[ip] - errMin

    dxp[ip] = dx[imin]
    dyp[ip] = dy[imin]
    dap[ip] = da[imin]
  end
  --Filter toward best matching landmark position:
  for ip = 1,N do
--print(string.format("%d %.1f %.1f %.1f",ip,xp[ip],yp[ip],ap[ip]))
    xp[ip] = xp[ip] + rFilter * (dxp[ip] - r * math.cos(ap[ip] + a))
    yp[ip] = yp[ip] + rFilter * (dyp[ip] - r * math.sin(ap[ip] + a))
    ap[ip] = ap[ip] + aFilter * dap[ip]

    -- check boundary
    xp[ip] = math.min(xMax, math.max(-xMax, xp[ip]))
    yp[ip] = math.min(yMax, math.max(-yMax, yp[ip]))
  end
end

---Update particles according to a goal detection
--@param pos All possible positions of the goals
--For example, each post location is an entry in pos
--@param v x and y coordinates of detected goal relative to robot
--function poseFilter.goal_observation(pos, v)
---------------------------------------------------------------------------
-- Now we have two ambiguous goals to check
-- So we separate the triangulation part and the update part
---------------------------------------------------------------------------

function poseFilter.triangulate(pos,v)
  --Based on old code

  -- Use angle between posts (most accurate)
  -- as well as combination of post distances to triangulate
  local aPost = {}
  aPost[1] = math.atan2(v[1][2], v[1][1])
  aPost[2] = math.atan2(v[2][2], v[2][1])
  local daPost = mod_angle(aPost[1]-aPost[2])

  -- Radius of circumscribing circle
  local sa = math.sin(math.abs(daPost))
  local ca = math.cos(daPost)
  local rCircumscribe = goalWidth/(2*sa)

  -- Post distances
  local d2Post = {}
  d2Post[1] = v[1][1]^2 + v[1][2]^2
  d2Post[2] = v[2][1]^2 + v[2][2]^2
  local ignore, iMin = util.min(d2Post)

  -- Position relative to center of goal:
  local sumD2 = d2Post[1] + d2Post[2]
  local dGoal = math.sqrt(.5*sumD2)
  local dx = (sumD2 - goalWidth^2)/(4*rCircumscribe*ca)
  local dy = math.sqrt(math.max(.5*sumD2-.25*goalWidth^2-dx^2, 0))

  -- Predicted pose:
  local x = pos[iMin][1]
  x = x - util.sign(x) * dx
  local y = pos[iMin][2]
  y = util.sign(y) * dy
  local a = math.atan2(pos[iMin][2] - y, pos[iMin][1] - x) - aPost[iMin]

  pose={}
  pose.x=x
  pose.y=y
  pose.a=a

  aGoal = util.mod_angle((aPost[1]+aPost[2])/2)

  return pose,dGoal,aGoal
end

triangulate2 = function (pos,v)

   local aPost = {}
   local d2Post = {}

   aPost[1] = math.atan2(v[1][2], v[1][1])
   aPost[2] = math.atan2(v[2][2], v[2][1])
   d2Post[1] = v[1][1]^2 + v[1][2]^2
   d2Post[2] = v[2][1]^2 + v[2][2]^2
   d1 = math.sqrt(d2Post[1])
   d2 = math.sqrt(d2Post[2])


   postfix=1
   --postfix=0

   if postfix>0 then

     if d1>d2 then
       --left post correction based on right post
       -- v1=kcos(a1),ksin(a1)
       -- k^2 - 2k(v[2][1]cos(a1)+v[2][2]sin(a1)) + d2Post[2]-goalWidth^2 = 0
       local ca = math.cos(aPost[1])
       local sa = math.sin(aPost[1])
       local b = v[2][1]*ca+ v[2][2]*sa
       local c = d2Post[2]-goalWidth^2

       if b*b-c>0 then
         -- vcm.add_debug_message("Correcting left post\n")
         -- vcm.add_debug_message(string.format("Left post angle: %d\n",aPost[1]*180/math.pi))

         k1=b-math.sqrt(b*b-c)
         k2=b+math.sqrt(b*b-c)

    --          vcm.add_debug_message(string.format("d1: %.1f v1: %.1f %.1f\n",
    --         d1,v[1][1],v[1][2]))
    --          vcm.add_debug_message(string.format("k1: %.1f v1_1: %.1f %.1f\n",
    -- k1,k1*ca,k1*sa ))
    --          vcm.add_debug_message(string.format("k2: %.1f v1_2: %.1f %.1f\n",
    -- k2,k2*ca,k2*sa ))

         if math.abs(d2-k1)<math.abs(d2-k2) then
  	        v[1][1],v[1][2]=k1*ca,k1*sa
         else
	          v[1][1],v[1][2]=k2*ca,k2*sa
         end
       end
     else
       --right post correction based on left post
       -- v2=kcos(a2),ksin(a2)
       -- k^2 - 2k(v[1][1]cos(a2)+v[1][2]sin(a2)) + d2Post[1]-goalWidth^2 = 0
       local ca=math.cos(aPost[2])
       local sa=math.sin(aPost[2])
       local b=v[1][1]*ca+ v[1][2]*sa
       local c=d2Post[1]-goalWidth^2

       if b*b-c>0 then
         k1=b-math.sqrt(b*b-c)
         k2=b+math.sqrt(b*b-c)

      --        vcm.add_debug_message(string.format("d2: %.1f v2: %.1f %.1f\n",
      --     d2,v[2][1],v[2][2]))
      --        vcm.add_debug_message(string.format("k1: %.1f v2_1: %.1f %.1f\n",
      -- k1,k1*ca,k1*sa ))
      --        vcm.add_debug_message(string.format("k2: %.1f v2_2: %.1f %.1f\n",
      -- k2,k2*ca,k2*sa ))

         if math.abs(d2-k1)<math.abs(d2-k2) then
            v[2][1],v[2][2]=k1*ca,k1*sa
         else
            v[2][1],v[2][2]=k2*ca,k2*sa
         end
       end
     end

   end

   --Use center of the post to fix angle
   vGoalX=0.5*(v[1][1]+v[2][1])
   vGoalY=0.5*(v[1][2]+v[2][2])
   rGoal = math.sqrt(vGoalX^2+vGoalY^2)

   if aPost[1]<aPost[2] then
     aGoal=-math.atan2 ( v[1][1]-v[2][1] , -(v[1][2]-v[2][2]) )
   else
     aGoal=-math.atan2 ( v[2][1]-v[1][1] , -(v[2][2]-v[1][2]) )
   end

   ca=math.cos(aGoal)
   sa=math.sin(aGoal)

   local dx = ca*vGoalX-sa*vGoalY
   local dy = sa*vGoalX+ca*vGoalY

   local x0 = 0.5*(pos[1][1]+pos[2][1])
   local y0 = 0.5*(pos[1][2]+pos[2][2])

   local x = x0 - util.sign(x0)*dx
   local y = -util.sign(x0)*dy
   local a=aGoal
   if x0<0 then a=mod_angle(a+math.pi) end
   local dGoal = rGoal

   pose={}
   pose.x=x
   pose.y=y
   pose.a=a

 --  aGoal = util.mod_angle((aPost[1]+aPost[2])/2)

   return pose,dGoal,aGoal
end

local function goal_triangulate_angle(pos,v,posea,debug)
--For this triangulation, we PRESEVER pose angle
--And we use the observaion only as angles (ignore distances)
  local aPost = {}  
  aPost[1] = math.atan2(v[1][2], v[1][1])
  aPost[2] = math.atan2(v[2][2], v[2][1])
  --line 1: (pose.x,pose.y) + ( cos(pos.a + aPost[1]), sin(pos.a+aPost[1])*t1
  --line 2: (pose.x,pose.y) + ( cos(pos.a + aPost[1]), sin(pos.a+aPost[1])*t2
  local c1 = math.cos(posea+aPost[1])
  local s1 = math.sin(posea+aPost[1])
  local c2 = math.cos(posea+aPost[2])
  local s2 = math.sin(posea+aPost[2])
  --Assumption: pos[1][1] = pos[2][1], pos[1][2] = -pos[2][2]
  local t1 = 2*pos[1][2] *c2 / (s1*c2-s2*c1)

  if debug and Config.debug.goal_localization then
    print("aPosts:",aPost[1]*180/math.pi,aPost[2]*180/math.pi)
    print("t1:",t1)
    print("t2:",t1*c1/c2)
  end
  
  local pose={}
  pose.x = pos[1][1] - c1*t1
  pose.y = pos[1][2] - s1*t1
  pose.a = posea

  local dGoal = math.sqrt((pose.x-pos[1][1])^2, pose.y^2)
  return pose, dGoal, t1
end



local function goal_observation_angle(pos1,pos2,v)
  local rFilter = rGoalFilter  
  for ip = 1,N do
    local pose1,dGoal1,t1 = goal_triangulate_angle(pos1,v,ap[ip],ip==1)
    local pose2,dGoal2,t2 = goal_triangulate_angle(pos2,v,ap[ip],ip==1)

    local rSigma1 = .25*dGoal1 + 0.20
    local rSigma2 = .25*dGoal2 + 0.20

    local xErr1 = pose1.x - xp[ip]
    local yErr1 = pose1.y - yp[ip]
    local rErr1 = math.sqrt(xErr1^2 + yErr1^2)
    local err1 = (rErr1/rSigma1)^2
    if t1<0 then err1 = math.huge end

    local xErr2 = pose2.x - xp[ip]
    local yErr2 = pose2.y - yp[ip]
    local rErr2 = math.sqrt(xErr2^2 + yErr2^2)    
    local err2 = (rErr2/rSigma2)^2
    if t2<0 then err2 = math.huge end

    --Filter towards best matching goal:
    if err1>err2 then
      wp[ip] = wp[ip] - err2
      xp[ip] = xp[ip] + rFilter*xErr2
      yp[ip] = yp[ip] + rFilter*yErr2
    else
      wp[ip] = wp[ip] - err1
      xp[ip] = xp[ip] + rFilter*xErr1
      yp[ip] = yp[ip] + rFilter*yErr1
    end
  end
end






local function goal_observation(pos1,pos2,v)

  if Config.use_angle_localization then
    return goal_observation_angle(pos1,pos2,v)
  end

  --Get pose estimate from two goalpost locations
  local pose1,dGoal1 = triangulate2(pos1,v)
  local pose2,dGoal2 = triangulate2(pos2,v)

  local x1,y1,a1=pose1.x,pose1.y,pose1.a
  local x2,y2,a2=pose2.x,pose2.y,pose2.a

  local rSigma1 = .25*dGoal1 + 0.20
  local rSigma2 = .25*dGoal2 + 0.20
  local aSigma = 5*math.pi/180
  local rFilter = rGoalFilter
  local aFilter = aGoalFilter

  for ip = 1,N do
    local xErr1 = x1 - xp[ip]
    local yErr1 = y1 - yp[ip]
    local rErr1 = math.sqrt(xErr1^2 + yErr1^2)
    local aErr1 = mod_angle(a1 - ap[ip])
    local err1 = (rErr1/rSigma1)^2 + (aErr1/aSigma)^2

    local xErr2 = x2 - xp[ip]
    local yErr2 = y2 - yp[ip]
    local rErr2 = math.sqrt(xErr2^2 + yErr2^2)
    local aErr2 = mod_angle(a2 - ap[ip])
    local err2 = (rErr2/rSigma2)^2 + (aErr2/aSigma)^2

    --Filter towards best matching goal:
    if err1>err2 then
      wp[ip] = wp[ip] - err2
      xp[ip] = xp[ip] + rFilter*xErr2
      yp[ip] = yp[ip] + rFilter*yErr2
      ap[ip] = ap[ip] + aFilter*aErr2
    else
      wp[ip] = wp[ip] - err1
      xp[ip] = xp[ip] + rFilter*xErr1
      yp[ip] = yp[ip] + rFilter*yErr1
      ap[ip] = ap[ip] + aFilter*aErr1
    end
  end
end


function poseFilter.post_both(v)
  goal_observation(goalUpper, goalLower, v)
end

function poseFilter.post_unknown(v)
  --TODO: this kills the angle-based localization for whatever reason!
  if Config.use_angle_localization then

  else
    landmark_observation(postAll, v[1], rUnknownPostFilter, aUnknownPostFilter)
  end
end

function poseFilter.post_left(v)
  landmark_observation(postLeft, v[1], rPostFilter, aPostFilter)
end

function poseFilter.post_right(v)
  landmark_observation(postRight, v[1], rPostFilter, aPostFilter)
end


---Updates particles according to the movement of the robot.
--Moves each particle the distance that the robot has moved
--since the last update.
--@param dx distance moved in x direction since last update
--@param dy distance moved in y direction since last update
--@param da angle turned since last update
function poseFilter.odometry(dx, dy, da)
  for ip = 1,N do
    ca = math.cos(ap[ip])
    sa = math.sin(ap[ip])
    xp[ip] = xp[ip] + dx*ca - dy*sa
    yp[ip] = yp[ip] + dx*sa + dy*ca
    ap[ip] = ap[ip] + da
  end
end

---Set all particles to x,y,a=0,0,0.
--This function poseFilter.does not update the weights
function poseFilter.zero_pose()
  xp = vector.zeros(N)
  yp = vector.zeros(N)
  ap = vector.zeros(N)
end



---Adds noise to particle x,y coordinates and angle.
function poseFilter.addNoise()
  da = 1.0*math.pi/180.0
  dr = 0.005
  xp = xp + dr * vector.new(util.randn(N))
  yp = yp + dr * vector.new(util.randn(N))
  --KAREN: noise in yaw makes localization worse
  -- ap = ap + da * vector.new(util.randn(N))
end

---Resample particles.
--If enough particles have low enough weights, then
--replaces low-weighted particles with new random particles
--and new particles that are nearby high-weighted particles
function poseFilter.resample()
  -- resample particles

  local wLog = {}
  for i = 1,N do
    -- cutoff boundaries
    wBounds = math.max(xp[i]-xMax,0)+math.max(-xp[i]-xMax,0)+
              math.max(yp[i]-yMax,0)+math.max(-yp[i]-yMax,0)
    wLog[i] = wp[i] - wBounds/0.1
    xp[i] = math.max(math.min(xp[i], xMax), -xMax)
    yp[i] = math.max(math.min(yp[i], yMax), -yMax)
  end

  --Calculate effective number of particles
  wMax, iMax = util.max(wLog)
  -- total sum
  local wSum = 0
  -- sum of squares
  local wSum2 = 0
  local w = {}
  for i = 1,N do
    w[i] = math.exp(wLog[i] - wMax)
    wSum = wSum + w[i]
    wSum2 = wSum2 + w[i]^2
  end

  local nEffective = (wSum^2) / wSum2
  if nEffective > .25*N then
    return
  end

  -- cum sum of weights
  -- wSum[i] = {cumsum(i), index}
  -- used for retrieving the sorted indices
  local wSum = {}
  wSum[1] = {w[1], 1}
  for i = 2,N do
     wSum[i] = {wSum[i-1][1] + w[i], i}
  end

  --normalize
  for i = 1,N do
    wSum[i][1] = wSum[i][1] / wSum[N][1]
  end

  --Add n more particles and resample high n weighted particles
  local rx = util.randu(N)
  local wSum_sz = #wSum
  for i = 1,N do
    table.insert(wSum, {rx[i], N+i})
  end

  -- sort wSum min->max
  table.sort(wSum, function(a,b) return a[1] < b[1] end)

  -- resample (replace low weighted particles)
  xp2 = vector.zeros(N)
  yp2 = vector.zeros(N)
  ap2 = vector.zeros(N)
  nsampleSum = 1
  ni = 1
  for i = 1,2*N do
    oi = wSum[i][2]
    if oi > N then
      xp2[ni] = xp[nsampleSum]
      yp2[ni] = yp[nsampleSum]
      ap2[ni] = ap[nsampleSum]
      ni = ni + 1
    else
      nsampleSum = nsampleSum + 1
    end
  end

  -- always put max particle
  xp2[1] = xp[iMax]
  yp2[1] = yp[iMax]
  ap2[1] = ap[iMax]

  xp = xp2
  yp = yp2
  ap = ap2
  wp = vector.zeros(N)
end

return poseFilter
