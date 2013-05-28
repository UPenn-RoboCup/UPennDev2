module(... or "", package.seeall)
require('unix')

initiated = 0
local count = 1

function update()
  if (initiated == 0) then
    t0 = unix.time()
    t1 = unix.time()
    initiated = 1
  else
    if (count % 100 == 0) then
      t1 = unix.time() 
      print ("Naoqi Frequency = "..(100/(t1-t0)).."Hz")
    end
    t0 = t1
    count = (count + 1) % 100
  end
end

