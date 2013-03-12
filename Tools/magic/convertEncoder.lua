require 'include'

local mat = require 'mat'

filePath = '../../../bus/proj4/'
fileName = filePath..'Encoders20.mat'

content = mat.load(fileName, true)
--content = mat.load('aa.mat', true)

for k, v in pairs(content[1]) do
  print(k, v)
end
