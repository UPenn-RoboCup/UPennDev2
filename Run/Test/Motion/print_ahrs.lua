dofile('../../include.lua')
require('dcm')
require('unix')
require('util')
require('pcm')

local ahrs1={}
while true do
  unix.sleep(1)
  --ahrs1 = dcm:get_ahrs()
  ahrs1 = pcm:get_l_foot_cop_pressure()
  --print('ahrs', ahrs1[4], ahrs1[5], ahrs1[7], ahrs1[8])
  print('ahrs', ahrs1[1], ahrs1[3], ahrs1[4], ahrs1[5])
end
