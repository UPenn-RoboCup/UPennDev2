dofile('../../include.lua')
require('dcm')
require('unix')
require('util')

local ahrs1={}
while true do
  unix.sleep(1)
  ahrs1 = dcm:get_ahrs()
  print('ahrs', ahrs1[4], ahrs1[5], ahrs1[7], ahrs1[8])
end
