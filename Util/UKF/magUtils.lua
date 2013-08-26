require 'include'
require 'common'
require 'poseUtils'

require 'torch'

function magCorrect(mag)
  local m = torch.DoubleTensor(3):fill(0)
  m[1] = mag.y
  m[2] = -mag.x
  m[3] = -mag.z

  return m
end

function calibrateMagnetometer(magset)
  -- AN4246 AN4248
  local sampleNum = 10000
  local divider = math.ceil(#magset / sampleNum)
  local Y = torch.DoubleTensor(sampleNum, 1):fill(0)
  local X = torch.DoubleTensor(sampleNum, 4):fill(1)
  
  local sampleCnt = 1
  for i = 1, #magset, divider do
    local mag = magCorrect(magset[i])
    local magNorm = mag[1]^2 + mag[2]^2 + mag[3]^2
    Y[sampleCnt][1] = magNorm
    X:narrow(1, sampleCnt, 1):narrow(2, 1, 3):copy(mag)
    sampleCnt = sampleCnt + 1
  end
  
  local Beta = torch.inverse(X:t() * X) * X:t() * Y
  local V = torch.DoubleTensor(3):copy(Beta:narrow(1,1,3) / 2)
  local B = torch.sqrt(Beta[4] * Beta[4] + V[1] * V[1] + V[2] * V[2] + V[3] * V[3])
  local P = (Y - X * Beta):t() * (Y - X * Beta)
  local M = sampleNum
  local epsi = torch.sqrt(P / M)
  epsi:div(2 * B * B)
  print('Calibration fit')
  print(epsi)
  
  print('V - hard iron offset')
  print(V)
  print('Geomagnetic field strength')
  print(B)
  return V, B
end

--local datasetpath = '../data/010213180247/'
--local datasetpath = '../data/010213192135/'
----local datasetpath = '../data/'
----local datasetpath = '../data/dataset9/'
--local magset = loadData(datasetpath, 'magPruned')
--calibrateMagnetometer(magset)

---- mag calibration value
V = torch.DoubleTensor({425.2790, 51.8208, -1299.8381})
B = 1076821.092515
--
V = torch.DoubleTensor({51.7819, 425.4612, 1300.3680})
B = 1077652.0811881


----local datasetpath = '../data/010213192135/'
----local magset = loadData(datasetpath, 'magPruned')
--V = torch.DoubleTensor({-273.0220, -183.1293, 403.2035})
--B = 5725.4947845329

local declinationAngle = -205.7/ 1000.0

function Mag2Heading(mag)
  local heading = math.atan2(mag[2], mag[1])
  heading = heading + declinationAngle
  if heading < 0 then heading = heading + 2 * math.pi end
--  if heading > 2 * math.pi then heading = heading - 2 * math.pi end
  return heading
end

function magCalibrated(mag)
  local _mag = torch.DoubleTensor(3):fill(0)
  _mag[1] = mag[1] - V[1]
  _mag[2] = mag[2] - V[2]
  _mag[3] = mag[3] - V[3]
  return _mag
end

function correctRange(angle)
  if angle < 0 then angle = angle + 2 * math.pi end
  if angle > 2 * math.pi then angle = angle - 2 * math.pi end
  return angle
end

function magTiltCompensate(mag, acc)
--  print('mag', mag)
--  print('acc', acc)
  -- AN4246 AN4247
  -- need -180 ~ 180
  local roll = math.atan2(acc[2][1], acc[3][1])
  -- need -90 ~ 90
  local tanPitch = -acc[1][1] / 
                  (acc[2][1]*math.sin(roll)+acc[3][1]*math.cos(roll))
  local pitch = math.atan(tanPitch)
--  local pitch = math.atan2(-acc[1][1] ,
--        (acc[2][1]*math.sin(roll)+acc[3][1]*math.cos(roll)))

  local Ry = rotY(pitch)
  local Rx = rotX(roll)
  local Bf = Ry * Rx * magCalibrated(mag)
  -- heading
  local yaw = math.atan2(-Bf[2], Bf[1])
  yaw = yaw + declinationAngle
  if yaw < 0 then yaw = yaw + 2 * math.pi end
  if yaw > 2 * math.pi then yaw = yaw - 2 * math.pi end

  return yaw, Bf
--  return mag - V
end


