--/usr/local/bin/luajit
--package.path = package.path .. ';shm/shm.lua'

local estop=require('estop')

estop.init()

estop.display(1,"test")

while true do
  ret = estop.update()  
  print("ESTOP:",ret.estop)
  print("LSTICK:",unpack(ret.lstick))
  print("RSTICK:",unpack(ret.rstick))
  print("LBUTTON:",unpack(ret.lbutton))
  print("RBUTTON:",ret.rbutton)
end
estop.close()