local dinast = require 'dinast'
local device = dinast.init()
print(device)

while true do
  img_ch1, img_ch2 = device:get_image() 
  print(img_ch1, img_ch2)
end
