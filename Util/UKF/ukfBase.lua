require 'include'
require 'common'
require 'torch'

require 'poseUtils'
require 'magUtils'
require 'imuUtils'
require 'matrixUtils'
util = require 'util'

-- state init Posterior state
state = torch.DoubleTensor(10, 1):fill(0) -- x, y, z, vx, vy, vz, q0, q1, q2, q3
state[7] = 1

-- state Cov, Posterior
P = torch.DoubleTensor(9, 9):fill(0) -- estimate error covariance
P:sub(1, 3, 1, 3):eye(3, 3):mul(0.05^2)
P:sub(4, 6, 4, 6):eye(3, 3):mul(0.01^2)
P:sub(7, 9, 7, 9):eye(3, 3):mul((10 * math.pi / 180)^2)

Q = torch.DoubleTensor(9, 9):fill(0) -- process noise covariance
Q:sub(1, 3, 1, 3):eye(3, 3):mul(0.05^2)
Q:sub(4, 6, 4, 6):eye(3, 3):mul(0.01^2)
Q:sub(7, 9, 7, 9):eye(3, 3):mul((10 * math.pi / 180)^2)

--R = torch.DoubleTensor(12, 12):fill(0) -- measurement noise covariance
posCovR = torch.DoubleTensor(3,3):eye(3, 3):mul(0.01^2)
velCovR = torch.DoubleTensor(3,3):eye(3, 3):mul(0.01^2)
qCovRG  = torch.DoubleTensor(3,3):eye(3, 3):mul((1 * math.pi / 180)^2)
qCovRM  = torch.DoubleTensor(3,3):eye(3, 3):mul((10 * math.pi / 180)^2)

ns = 9
Chi = torch.DoubleTensor(10, 2 * ns):fill(0)
Chi:narrow(1, 7, 1):fill(1)
--util.ptorch(Chi)
ChiMean = torch.DoubleTensor(10, 1):fill(0)
ChiMean:narrow(1, 7, 1):fill(1)
e = torch.DoubleTensor(3, 2 * ns):fill(0)
Y = torch.DoubleTensor(10, 2 * ns):fill(0)
yMean = torch.DoubleTensor(10, 1):fill(0)

-- Imu Init
accBias = torch.DoubleTensor({-0.03, 0, 0})

acc = torch.DoubleTensor(3, 1):fill(0)
gacc = torch.DoubleTensor(3, 1):fill(0)
rawacc = torch.DoubleTensor(3, 1):fill(0)
gyro = torch.DoubleTensor(3, 1):fill(0)
g = torch.DoubleTensor(3, 1):fill(0)
gInitCount = 0
gInitCountMax = 100

processInit = false
imuInit = false
gpsInit = false 
magInit = false
velInit = false
gravity = 9.80
imuTstep = 0
gpsTstep = 0

KGainCount = 0

function imuInitiate(step, accin)
  if not imuInit then
    g:add(accin)
    gInitCount = gInitCount + 1
    if gInitCount >= gInitCountMax then
      g = g:div(gInitCount)
      print('Initiated Gravity')
      return true
    else return false end
  end
end


function processUpdatePos(tstep, gps)
  if not imuInit or not magInit or not gpsInit then return false end

  local dtime = tstep - gpsTstep
  gpsTstep = tstep
  if not velInit then 
    dtime = 0 
    velInit = true
  end

  local res = GenerateSigmaPoints(dtime)
  local speed = gps.nspeed * 0.514444
  -- Process Model Update and generate y
  local rpy = Quat2rpy(state:narrow(1, 7, 4))
  local yaw = rpy[3]
  for i = 1, 2 * ns do
    local Chicol = Chi:narrow(2, i, 1)
 -- x
    Y[1][i] = Chicol[1][1] + speed * math.cos(yaw) * dtime
  -- y
    Y[2][i] = Chicol[2][1] + speed * math.sin(yaw) * dtime
  end
 -- Y mean
  yMean:copy(torch.mean(Y, 2))
  yMeanQ = QuatMean(Y:narrow(1, 7, 4), state:narrow(1, 7, 4))
  yMean:narrow(1, 7, 4):copy(yMeanQ)
--  print(yMean[1][1], yMean[2][1], yMean[3][1])

  res = PrioriEstimate(dtime)

  processInit = true
  return true
end

function processUpdateRot(tstep, imu)
  rawacc, gyro = imuCorrent(imu, accBias)
--  gacc = accTiltCompensate(rawacc)
  local currentQ = state:narrow(1, 7, 4)
  local R = Quat2R(currentQ)
  gacc = R * rawacc
  local dtime = tstep - imuTstep
  imuTstep = tstep

  if not imuInit then
    imuInit = imuInitiate(tstep, gacc)
    return false
  end
  
  if not magInit or not gpsInit then return false end
--  if not processInit then return false
--  end

  -- substract gravity from z axis and convert from g to m/s^2
  acc:copy(gacc - g)
  acc = acc * gravity
  local res = GenerateSigmaPoints(dtime)
  if res == false then return false end
  res = ProcessModelRot(dtime)
  if res == false then return false end
  res = PrioriEstimate(dtime)
  if res == false then return false end

  processInit = true
  return true
end

function GenerateSigmaPoints(dt)
  -- Sigma points
  local W = cholesky((P+Q):mul(math.sqrt(2*ns)))
  local q = state:narrow(1, 7, 4)
  for i = 1, ns do
    -- Sigma points for pos and vel
    Chi:sub(1,6,i,i):copy(state:sub(1,6) + W:sub(1,6,i,i))
    Chi:sub(1,6,i+ns,i+ns):copy(state:sub(1,6) - W:sub(1,6,i,i))

    -- Sigma points for Quaternion
    local eW = W:narrow(2, i, 1):narrow(1, 7, 3)
    local qW = Vector2Quat(eW)
    Chi:narrow(2, i, 1):narrow(1, 7, 4):copy(QuatMul(q, qW))
    qW = Vector2Quat(-eW)
    Chi:narrow(2, i + ns, 1):narrow(1, 7, 4):copy(QuatMul(q, qW))
  end
  return true
end

function ProcessModelRot(dt)
  -- Process Model Update and generate y
--  local F = torch.DoubleTensor({{1,0,0,dt,0,0}, {0,1,0,0,dt,0}, {0,0,1,0,0,dt},
--                          {0,0,0,1,0,0}, {0,0,0,0,1,0}, {0,0,0,0,0,1}})
  local G = torch.DoubleTensor({{dt^2/2,0,0}, {0,dt^2/2,0}, {0,0,dt^2/2},
                          {dt,0,0}, {0,dt,0}, {0,0,dt}})
  -- Y
  for i = 1, 2 * ns do
    local Chicol = Chi:narrow(2, i, 1)
--    Y:narrow(2, i, 1):narrow(1, 1, 6):copy(F * Chicol:narrow(1, 1, 6) + G * acc)

    local q = Chicol:narrow(1, 7, 4)
    local dq = Vector2Quat(gyro, dt)
    Y:narrow(2, i, 1):narrow(1, 7, 4):copy(QuatMul(q,dq))
  end
 -- Y mean
  yMean:copy(torch.mean(Y, 2))
  yMeanQ = QuatMean(Y:narrow(1, 7, 4), state:narrow(1, 7, 4))
  yMean:narrow(1, 7, 4):copy(yMeanQ)
  return true
end

function PrioriEstimate(dt)
  -- Generate priori estimate state and covariance
  -- priori state = mean(Y)
  state:copy(yMean)
  local PPriori = torch.DoubleTensor(9, 9):fill(0)
  for i = 1, 2 * ns do
    local Ycol = Y:narrow(2, i, 1)
    local WDiff = torch.DoubleTensor(9, 1):fill(0)
    -- Pos & Vel
    WDiff:narrow(1, 1, 6):copy(Ycol:narrow(1, 1, 6) - yMean:narrow(1, 1, 6))
    -- Rotation
    local YqDiff = QuatMul(Ycol:narrow(1, 7, 4), QuatInv(yMean:narrow(1, 7, 4)))
    WDiff:narrow(1, 7, 3):copy(Quat2Vector(YqDiff))
    PPriori:add(WDiff * WDiff:t())
  end
  PPriori:div(2.0 * ns)
  P:copy(PPriori)
  return true
end

function KalmanGainUpdate(Z, zMean, v, R)
  -- Pxz Pzz Pvv
  local Pxz = torch.DoubleTensor(9, zMean:size(1)):fill(0)
  local Pzz = torch.DoubleTensor(zMean:size(1), zMean:size(1)):fill(0)
  for i = 1, 2 * ns do
    local Ycol = Y:narrow(2, i, 1)
    local WDiff = torch.DoubleTensor(9, 1):fill(0)
    -- Pos & Vel
    WDiff:narrow(1, 1, 6):copy(Ycol:narrow(1, 1, 6) - yMean:narrow(1, 1, 6))
    -- Rotation
    local YqDiff = QuatMul(Ycol:narrow(1, 7, 4), QuatInv(yMean:narrow(1, 7, 4)))
    WDiff:narrow(1, 7, 3):copy(Quat2Vector(YqDiff))

    local ZDiff = Z:narrow(2, i, 1) - zMean
    Pxz:add(WDiff * ZDiff:t())
    Pzz:add(ZDiff * ZDiff:t())
  end
  Pxz:div(2 * ns)
  Pzz:div(2 * ns)
  local Pvv = Pzz + R

  -- K
  local K = Pxz * torch.inverse(Pvv)

  -- posterior
  local stateadd = K * v
  state:narrow(1, 1, 6):add(stateadd:narrow(1, 1, 6))
  local stateqi = state:narrow(1, 7, 4)
  local stateaddqi = Vector2Quat(stateadd:narrow(1, 7, 3))
  state:narrow(1, 7, 4):copy(QuatMul(stateqi, stateaddqi))
  P = P - K * Pvv * K:t()
  KGainCount = KGainCount + 1
  return true
end

function measurementGravityUpdate()
  if not processInit then return false end

  local gq = torch.DoubleTensor({0,0,0,1})
  local Z = torch.DoubleTensor(3, 2 * ns):fill(0)
  for i = 1, 2 * ns do
    local Zcol = Z:narrow(2, i, 1)
    local Chicol = Chi:narrow(2, i , 1)
    local qk = Chicol:narrow(1, 7, 4)
    Zcol:copy(QuatMul(QuatMul(qk, gq), QuatInv(qk)):narrow(1, 2, 3))
  end
  local zMean = torch.mean(Z, 2)
  local v = rawacc - zMean
  local R = qCovRG
  return KalmanGainUpdate(Z, zMean, v, R)
end

-- Geo Init
function gpsInitiate(gps)
  local gpspos = torch.DoubleTensor({gps.x, gps.y, gps.z})
  -- set init pos
  state:sub(1, 3, 1, 1):copy(gpspos)
  print('initiate GPS') 
  return true
end

function measurementGPSUpdate(gps)
  if not imuInit then return false end

  if not gpsInit then
    gpsInit = gpsInitiate(gps)
    return false
  end

  gpspos =torch.DoubleTensor({gps.x, gps.y, gps.z})
  -- require DOP to adjust measurement cov
  local HDOP = gps.HDOP
  if HDOP > 1.0 then HDOP = 1.0 end
  local VDOP = gps.VDOP
  if VDOP > 2.0 then VDOP = 2.0 end
  local PDOP = gps.PDOP
  local Satellites = gps.satellites
  gpsdop = torch.DoubleTensor({gps.HDOP, gps.VDOP, gps.PDOP})
  local pDOP = 0.117 * Satellites - 0.37
  if pDOP < 0 then pDOP = 0 end
  if pDOP > 1 then pDOP = 1 end
  local rHDOP = ( math.sqrt(HDOP^2/2) * (1 - pDOP) )^2
  local rVDOP = ( VDOP * (1 - pDOP) )^2
  local rPDOP = ( PDOP * (1 - pDOP) )^2

  local R = torch.DoubleTensor(3,3):eye(3, 3):mul(0.07^2)
  R[1][1] = rHDOP / 100
  R[2][2] = rHDOP / 100
  R[3][3] = rVDOP / 50

  if not processInit then return false end

  local Z = torch.DoubleTensor(3, 2 * ns):fill(0)
  for i = 1, 2 * ns do
    local Zcol = Z:narrow(2, i, 1)
    local Chicol = Chi:narrow(2, i , 1)
    Zcol:copy(Chicol:narrow(1, 1, 3))
  end
  local zMean = torch.mean(Z, 2)

  -- reset Z with zMean since no measurement here
  local v = gpspos - zMean

  -- linear measurement update
  local C = torch.DoubleTensor(3, 9):fill(0)
  C:narrow(1, 1, 3):narrow(2, 1, 3):eye(3, 3)
  local K = P * C:t() * torch.inverse(C * P * C:t() + R)
  state:narrow(1, 1, 3):copy(state:narrow(1, 1, 3) + K:narrow(1,1,3):narrow(2,1,3) * v)
  P = (torch.DoubleTensor(9,9):eye(9,9) - K * C) * P

  KGainCount = KGainCount + 1
--  return KalmanGainUpdate(Z, zMean, v, R)
  return true

end

function magInitiate(mag)
  -- mag and imu coordinate x, y reverse
  local rawmag = magCorrect(mag)
  -- calibrated & tilt compensated heading 
  local heading, Bf = magTiltCompensate(rawmag, rawacc)

  -- HACK here as set mag heading always as init yaw value
  local initRPY = torch.DoubleTensor({0,0,heading}) 
  local initQ = torch.DoubleTensor(rpy2Quat(initRPY))
  state:narrow(1, 7, 4):copy(initQ)
  print('initiated mag')
  return true  
end

function measurementMagUpdate(mag)
  if not imuInit then return false end

  -- mag and imu coordinate x, y reverse
  local rawmag = magCorrect(mag)
  -- calibrated & tilt compensated heading 
  local heading, Bf = magTiltCompensate(rawmag, rawacc)

  -- set init orientation
  if not magInit then magInit = magInitiate(mag) return false end

  if not processInit then return false end

  local calibratedMag = magCalibrated(rawmag)
  local heading, Bf = magTiltCompensate(rawmag, rawacc)
--  Bf:div(Bf:norm())
--  util.ptorch(Bf)
--  local Z = torch.DoubleTensor(4, 2 * ns):fill(0)
--  for i = 1, 2 * ns do
--    local Zcol = Z:narrow(2, i, 1)
--    local Chicol = Chi:narrow(2, i, 1)
--    Zcol:copy(Chicol:narrow(1, 7, 4))
--  end
--  local zMeanQ = QuatMean(Z, state:narrow(1, 7, 4))
--  local zMeanRPY = Quat2rpy(zMeanQ)
--  local v = heading - zMeanRPY[3]
--  local R = (0.1)^2
--  -- linear measurement update
--  local C = torch.DoubleTensor(1, 9):fill(0)
--  C[1][9] = 1
--  local K = P * C:t() * torch.inverse(C * P * C:t() + R)
--  local newYaw = zMeanRPY[3] + K[9][1] * v
--  local newRPY = torch.DoubleTensor({zMeanRPY[1], zMeanRPY[2], newYaw})
--  local newQ = rpy2Quat(newRPY)
--  state:narrow(1, 7, 4):copy(newQ)
--  P = (torch.DoubleTensor(9,9):eye(9,9) - K * C) * P
--  KGainCount = KGainCount + 1

--  local mq = torch.DoubleTensor({0,1,0,0})
--  local Z = torch.DoubleTensor(3, 2 * ns):fill(0)
--  for i = 1, 2 * ns do
--    local Zcol = Z:narrow(2, i, 1)
--    local Chicol = Chi:narrow(2, i , 1)
--    local qk = Chicol:narrow(1, 7, 4)
--    Zcol:copy(QuatMul(QuatMul(qk, mq), QuatInv(qk)):narrow(1, 2, 3))
--  end
--  local zMean = torch.mean(Z, 2)
--  local v = Bf:narrow(1, 3, 1) - zMean:narrow(1, 3, 1)
--  local R = (0.1)^2
--  return KalmanGainUpdate(Z:narrow(1, 3, 1), zMean:narrow(1, 3, 1), v, R)

end
