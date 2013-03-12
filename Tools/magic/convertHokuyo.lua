require 'include'
require 'common'

local serialization = require 'serialization'

local mat = require 'mat'

filePath = '../../../bus/proj4/'

local HokuyoFileList = assert(io.popen('/bin/ls '..filePath..'Hokuyo2*'))
local HokuyoFileNum = 0
for line in HokuyoFileList:lines() do
  HokuyoFileNum = HokuyoFileNum + 1
end

for i = 1, 1 do
--for i = 1, 4 do
  local fileName = filePath..'Hokuyo2'..i..'.mat'
  local content = mat.load(fileName, true)

  for k, v in pairs(content[1].Hokuyo0) do
    print(k, v)
  end
  local Hokuyo = content[1].Hokuyo0

--  local counts = content[1].Hokuyos.counts['']
--  local ts = content[1].Hokuyos.ts['']
--  local Hokuyo = {}
--  for i = 1, counts:size(2) do
--    local hokuyo = {['num1'] = counts[1][i], ['num2'] = counts[2][i],
--                      ['num3'] = counts[3][i], ['num4'] = counts[4][i],
--                      ['timestamp'] = ts[1][i]}
--    Hokuyo[#Hokuyo + 1] = hokuyo
--  end
--  
--  saveData(Hokuyo, 'Hokuyo2'..i, filePath)
end
