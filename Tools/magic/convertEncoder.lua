require 'include'
require 'common'

local serialization = require 'serialization'

local mat = require 'mat'

filePath = '../../../bus/proj4/'

local EncoderFileList = assert(io.popen('/bin/ls '..filePath..'Encoder2*'))
local EncoderFileNum = 0
for line in EncoderFileList:lines() do
  EncoderFileNum = EncoderFileNum + 1
end

for i = 1, 4 do
  local fileName = filePath..'Encoders2'..i..'.mat'
  local content = mat.load(fileName, false)
  local counts = content[1].Encoders.counts['']
  local ts = content[1].Encoders.ts['']
  local Encoder = {}
  for i = 1, counts:size(2) do
    local encoder = {['num1'] = counts[1][i], ['num2'] = counts[2][i],
                      ['num3'] = counts[3][i], ['num4'] = counts[4][i],
                      ['timestamp'] = ts[1][i]}
    Encoder[#Encoder + 1] = encoder
  end
  
  saveData(Encoder, 'Encoder2'..i, filePath)
end
