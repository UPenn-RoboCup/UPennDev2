dofile'../../include.lua'

uvc2 = require'uvc2'
cam = uvc2.init()
unix.usleep(1e6)
mt = getmetatable(cam)
print("Cam and metatable", cam, mt)

for k, v in pairs(uvc2) do
  print("uvc2",k,v)
end

for k, v in pairs(mt) do
  print("metatable",k,v)
end

print()
print('Get params...')
cam:get_param()

unix.usleep(1e6)

print()
print('Set parameters...')
cam:set_param()

print()
print('Get params...')
cam:get_param()

print()
print("Grab frames...")
cam:stream_on()

print("Loop")
for i=1,10 do
  img, size, count, time = cam:get_frame()
  print(count, img, size, time)
  unix.usleep(1e6/30)
end
--[[
print("Stream off...")
cam:stream_off()
--]]
--[[
print('Close...')
cam:close()
unix.usleep(1e4)
--]]

jpeg = require'jpeg'
cy = jpeg.compressor('yuyv')
img, size, count, time = cam:get_frame()
if size>0 then
  jimg = cy:compress(img, 640, 480)
  print("COMPRESSED", #jimg)
  f = io.open("jimg_uvc2.jpeg", 'w')
  f:write(jimg)
  f:close()
end


print('Done')