dofile'../../include.lua'

uvc2 = require'uvc2'
cam = uvc2.init()
mt = getmetatable(cam)
print("Cam and metatable", cam, mt)

for k, v in pairs(uvc2) do
  print("uvc2",k,v)
end

for k, v in pairs(mt) do
  print("metatable",k,v)
end

cam:stream_on()

unix.usleep(1e6)

cam:stream_off()
cam:close()
