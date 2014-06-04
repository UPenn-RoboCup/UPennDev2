local HeadTransform={}

local Transform = require'Transform'
local vector = require'vector'
require'vcm'
require'mcm'
local Body = require'Body'

--Neck and camera transform
tHead = Transform.eye()
tNeck = Transform.eye()

pitchMin = Config.head.pitchMin
pitchMax = Config.head.pitchMax
yawMin = Config.head.yawMin
yawMax = Config.head.yawMax
cameraPos = Config.head.cameraPos
cameraAngle = Config.head.cameraAngle
neckX    = Config.head.neckX 
neckZ    = Config.head.neckZ 

labelA = {}
--[[
if( webots ) then
  labelA.m = Config.camera.width
  labelA.n = Config.camera.height
else
  labelA.m = Config.camera.width/2
  labelA.n = Config.camera.height/2
end
--]]


local horizonA, horizonB, horizonDir
local x0A,x0B,y0A,y0B,focalA,focalB

local function getCameraOffset() 
    local v=vector.new({0,0,0,1})
    v=tHead*v
    v=v/v[4]
    return v
end

--[[
local function getNeckOffset() 
  local v=vector.new({0,0,0,1})
  v=tNeck*v
  v=v/v[4]
  return v
end
--]]


-- Get updated neck offset w/o running update
local function getNeckOffset()
  --TODO: read current position rather than command position
  local bodyHeight = mcm.get_stance_bodyHeight()
  local bodyTilt = mcm.get_stance_bodyTilt()

  local footX = 0.05  --TODO
  local tNeck0 = Transform.trans(-footX,0,bodyHeight) 
  tNeck0 = tNeck0*Transform.rotY(bodyTilt)
  tNeck0 = tNeck0*Transform.trans(neckX,0,neckZ)
  local v = vector.new({0,0,0,1})
  v = tNeck0*v
  v = v/v[4]
  return v
end


function HeadTransform.coordinatesA(c, scale)
  scale = scale or 1
  local v = vector.new({focalA,
                       -(c[1] - x0A),
                       -(c[2] - y0A),
                       scale})
  v = tHead*v
  v = v/v[4]
  --TODO: hack
  v[1] = v[1]*0.8
  v[2] = v[2]*1.9
  return v
end

function HeadTransform.coordinatesB(c, scale)
  scale = scale or 1
  local v = vector.new({focalB,
                        -(c[1] - x0B),
                        -(c[2] - y0B),
                        scale})
  v = tHead*v
  v = v/v[4]
  return v
end



function HeadTransform.update(headAngles,labelA,labelB)
  
  local scaleA = Config.vision.scaleA
  nxA = labelA.m
  x0A = 0.5 * (nxA-1)
  nyA = labelA.n
  y0A = 0.5 * (nyA-1)
  focalA = Config.camera.head.focal_length/
        (Config.camera.head.focal_base/nxA)

  scaleB = Config.vision.scaleB  
  nxB = labelB.m
  x0B = 0.5 * (nxB-1)
  nyB = labelB.n
  y0B = 0.5 * (nyB-1)
  focalB = focalA/scaleB

  --Now bodyHeight, Tilt, camera pitch angle bias are read from vcm 
  local bodyHeight = mcm.get_stance_bodyHeight()
  local bodyTilt = mcm.get_stance_bodyTilt()
  local pitch0 = 0
  --pitch0 = mcm.get_headPitchBias() --robot specific camera bias
  
  --Reference frame for everything else: pelvis frmae 
  --Reference frame for vision: the ground (to make things simple)
  --we consider waist angle to be zero here

  --tNeck = Transform.rotZ(waist[1])*Transform.rotY(waist[2])
  --TODO: footX, waistBias?
  tNeck = Transform.trans(0,0,bodyHeight)
  tNeck = tNeck*Transform.rotY(bodyTilt)
  tNeck = tNeck*Transform.trans(neckX,0,neckZ)
  --TODO: use command head angle is not good
  tNeck = tNeck*Transform.rotZ(headAngles[1])*Transform.rotY(headAngles[2]+pitch0)
  tHead = tNeck*Transform.trans(cameraPos[1], cameraPos[2], cameraPos[3])
  tHead = tHead*Transform.rotY(cameraAngle[2])

  --update camera position
  local vHead=vector.new({0,0,0,1})
  vHead=tHead*vHead
  vHead=vHead/vHead[4]
  vcm.set_head_camera_height(vHead[3])

  -- update horizon
  pa = headAngles[2] + cameraAngle[2] -- + bodyTilt   --TODO
  horizonA = (labelA.n/2.0) - focalA*math.tan(pa) - 2
  horizonA = math.min(labelA.n, math.max(math.floor(horizonA), 0)) 
  horizonB = (labelB.n/2.0) - focalB*math.tan(pa) - 1 
  horizonB = math.min(labelB.n, math.max(math.floor(horizonB), 0)) 
  
  --[[ Update horizon direction
  local ref = vector.new({0,1,0,1}) 
  local p0 = vector.new({0,0,0,1}) 
  local ref1 = vector.new({0,-1,0,1}) 
  p0 = tHead*p0 
  ref = tHead*ref  - p0
  ref1 = tHead*ref1 - p0
   -- print(ref,' ',ref1) 
  horizonDir = math.atan2(ref1[3],math.sqrt(ref1[1]^2+ref1[2]^2)) 
  --print('horizion angle: '..horizonDir*180/math.pi) 
  --]]
end


--Project 3d point to level plane with some height
--@param targetheight The heigh of horizontal plane to project onto
--@param v 3 dimensional point to project
function HeadTransform.projectGround(v,targetheight)

  targetheight=targetheight or 0
  local cameraOffset=getCameraOffset()
  local vout=vector.new(v)

  --Project to plane
  -- if v[3]<targetheight then
  if cameraOffset[3]>targetheight then 
    vout = cameraOffset+
      (v-cameraOffset)*(
         (cameraOffset[3]-targetheight) / (cameraOffset[3] - v[3] )
      )
  end    

  --Discount body offset
  --[[
  uBodyOffset = mcm.get_walk_bodyOffset()
  vout[1] = vout[1] + uBodyOffset[1]
  vout[2] = vout[2] + uBodyOffset[2]
  --]]
  return vout
end


--Camera IK without headangle limit
local function ikineCam0(x0,y0,z0)
  local bodyHeight = mcm.get_stance_bodyHeight()
  local bodyTilt = mcm.get_stance_bodyTilt()
  local pitch0 = 0
  --Look at ground by default
  z0 = z0 or 0

  --Cancel out the neck X and Z offset 
  -- x0 = x0 - neckZ*math.sin(bodyTilt) - neckX*math.cos(bodyTilt)
  -- z0 = z0 - (bodyHeight + math.cos(bodyTilt)*neckZ)
  
  local vNeck = getNeckOffset()
  x0 = x0 - vNeck[1]
  z0 = z0 - vNeck[3]

  --Cancel out body tilt angle
  v = Transform.rotY(-bodyTilt)*vector.new({x0,y0,z0,1})
  v=v/v[4]

  local x,y,z = v[1],v[2],v[3]
  -- x = x - cameraPos[1]
  local yaw = math.atan2(y, x)

  local norm = math.sqrt(x^2 + y^2 + z^2)

  --new IKcam that takes camera offset into account
  -------------------------------------------------------------
  -- sin(pitch)x + cos (pitch) z = c , c=camera z offset
  -- pitch = atan2(x,z) - acos(b/r),  r= sqrt(x^2+z^2)
  -- r*sin(pitch) = z *cos(pitch) + c, 
  -------------------------------------------------------------
  local c = cameraPos[3]
  local r = math.sqrt(x^2+y^2)
  local d = math.sqrt(r^2+z^2)
  local p0 = math.atan2(r,z) - math.acos(c/(d + 1E-10))

  local pitch = p0 - cameraAngle[2]- pitch0
  
  -- TODO: test
  -- pitch = math.asin(-z/(norm + 1E-10))
  
  return yaw, pitch
end

function HeadTransform.ikineCam(x, y, z)
  yaw,pitch=ikineCam0(x,y,z)
  yaw = math.min(math.max(yaw, yawMin), yawMax)
  pitch = math.min(math.max(pitch, pitchMin), pitchMax)
  return yaw,pitch
end

function HeadTransform.get_horizonA()
  return horizonA
end

function HeadTransform.get_horizonB()
  return horizonB
end

function HeadTransform.get_horizonDir()
  return horizonDir
end















--[[

function rayIntersectA(c)
  local p0 = vector.new({0,0,0,1.0})
  local p1 = vector.new({focalA,-(c[1]-x0A),-(c[2]-y0A),1.0})

  outrange = 0
  p1 = tHead * p1
  local p0 = tNeck * p0
  local v = p1 - p0
  -- if t < 0, the x value will be projected behind robot, simply reverse it
  -- since it is always very far away
  if (t < 0) then
    t = -t
  end
  local t = -p0[3]/v[3]
   -- if t < 0, the x value will be projected behind robot, simply reverse it
  -- since it is always very far away
  if (t < 0) then
    t = -t
    outrange = 1
  end 
  local p = p0 + t * v
  local uBodyOffset = mcm.get_walk_bodyOffset()
  p[1] = p[1] + uBodyOffset[1]
  p[2] = p[2] + uBodyOffset[2]
  return p, outrange
end


function rayIntersectB(c)
  local p0 = vector.new({0,0,0,1.0})
  local p1 = vector.new({focalB,-(c[1]-x0B),-(c[2]-y0B),1.0})

  outrange = 0
  p1 = tHead * p1
  local p0 = tNeck * p0
  local v = p1 - p0
  local t = -p0[3]/v[3]
  -- if t < 0, the x value will be projected behind robot, simply reverse it
  -- since it is always very far away
  if (t < 0) then
    t = -t
    outrange = 1
  end
  local p = p0 + t * v
  local uBodyOffset = mcm.get_walk_bodyOffset()
  p[1] = p[1] + uBodyOffset[1]
  p[2] = p[2] + uBodyOffset[2]
  return p, outrange
end

function exit()
end





function getCameraRoll()
  --Use camera IK to calculate how much the image is tilted
  headAngles = Body.get_head_position()
  r=3.0z0=0z1=0.7
  x0=r*math.cos(headAngles[1])
  y0=r*math.sin(headAngles[1])
  yaw1, pitch1=ikineCam0(x0,y0,z0,bottom)
  yaw2, pitch2=ikineCam0(x0,y0,z1,bottom)
  tiltAngle = math.atan( (yaw2-yaw1)/(pitch1-pitch2) ) 
  return tiltAngle
end


--]]



return HeadTransform
