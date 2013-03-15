require 'include'
require 'common'

local serialization = require 'serialization'

local mat = require 'mat'

filePath = '../../../bus/proj4/'

local imuFileList = assert(io.popen('/bin/ls '..filePath..'imuRaw2*'))
local imuFileNum = 0
for line in imuFileList:lines() do
  imuFileNum = imuFileNum + 1
end

local AccSen = 330
local GyrSen = 3.44
 

for i = 1, 4 do
--for i = 1, 1 do
  local fileName = filePath..'imuRaw2'..i..'.mat'
  local content = mat.load(fileName, false)

  util.ptable(content)
  local vals = content[1].vals
  local ts = content[2].ts
  assert(vals:size(2) == ts:size(2))
  imu = {}
  for i = 1, vals:size(2) do
    imuvals = vals:narrow(2, i, 1)
    tsvals = ts:narrow(2, i, 1)
    imu[i] = {}
    imu[i]['ax'] = imuvals[1][1]
    imu[i]['ay'] = imuvals[2][1]
    imu[i]['az'] = imuvals[3][1]
    imu[i]['wx'] = imuvals[4][1]
    imu[i]['wy'] = imuvals[5][1]
    imu[i]['wz'] = imuvals[6][1]
    imu[i]['timestamp'] = tsvals[1][1]
    imu[i]['type'] = 'imu' 
  end

  local imuMean = {0, 0, 0, 0, 0, 0}
  for i = 1, #imu do
    imuMean[1] = imuMean[1] + imu[i].ax / #imu
    imuMean[2] = imuMean[2] + imu[i].ay / #imu
    imuMean[3] = imuMean[3] + imu[i].az / #imu
    imuMean[4] = imuMean[4] + imu[i].wx / #imu
    imuMean[5] = imuMean[5] + imu[i].wy / #imu
    imuMean[6] = imuMean[6] + imu[i].wz / #imu
  end
  for i = 1, #imu do
    -- Substract bias and multiply gain
    imu[i].ax = (imu[i].ax - imuMean[1]) * 3300 / 1023 / AccSen
    imu[i].ay = (imu[i].ay - imuMean[2]) * 3300 / 1023 / AccSen
    imu[i].az = (imu[i].az - imuMean[3]) * 3300 / 1023 / AccSen
    imu[i].wx = (imu[i].wx - imuMean[4]) * 3300 / 1023 / GyrSen * math.pi / 180
    imu[i].wy = (imu[i].wy - imuMean[5]) * 3300 / 1023 / GyrSen * math.pi / 180
    imu[i].wz = (imu[i].wz - imuMean[6]) * 3300 / 1023 / GyrSen * math.pi / 180
    imu[i]['type'] = 'imu'
  end

--  local IMU = {}
--  for i = 1, #imu do
--    imustr = serialization.serialize(imu[i])
--    print(imustr)
--    local imu = {['num1'] = counts[1][i], ['num2'] = counts[2][i],
--                      ['num3'] = counts[3][i], ['num4'] = counts[4][i],
--                      ['timestamp'] = ts[1][i]}
--    IMU[#IMU + 1] = imu
--  end
--  
  saveData(imu, 'imu2'..i, filePath)
end
