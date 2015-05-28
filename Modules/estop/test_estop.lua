--/usr/local/bin/luajit
--package.path = package.path .. ';shm/shm.lua'

local estop=require('estop')

estop.init()
while true do
  ret = estop.update()  
  if ret==5 then
  	print("ESTOPPED!!!!!")
  end
end
estop.close()