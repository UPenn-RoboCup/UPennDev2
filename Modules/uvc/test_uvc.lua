local camera = require'uvc'
--local carray = require'carray'
video_ud1 = camera.init('/dev/video0', 640, 480, 'yuyv',1, 30)
--video_ud2 = camera.init('/dev/video1', 640, 480, 'yuyv')

while (true) do
  local img1, size1, count1, time1 = video_ud1:get_image();
--  local img2, size2, count2, time2 = video_ud2:get_image();

  if (img1 ~= -1) then
    print('img1', img1, size1, time1, count1)
  end
--  if (img2 ~= -1) then
--    print('img2', img2, size2, time2, count2)
--  end
end

--video_ud2 = uvc.init('/dev/video1', 320, 240, 'yuyv')
--video_ud3 = uvc.init('/dev/video2', 640, 480, 'yuyv')
--video_ud4 = uvc.init('/dev/video3', 640, 480, 'mjpeg')
--video_ud5 = uvc.init('/dev/video4', 640, 480, 'mjpeg')

--[[
local file1 = io.open('image1.jpg', 'w')
local counter = 0
while (true) do

    local img1, size1 = video_ud1:get_raw();
--    local img2, size2 = video_ud2:get_raw();
--    local img3 = video_ud3:get_raw();
--    local img4 = video_ud4:get_raw();
--    local img5 = video_ud5:get_raw();

    if (img1 ~= -1) then
        counter = counter + 1
        if (counter == 10) then
            video_ud1:close()
            video_ud1 = uvc.init('/dev/video0', 640, 480, 'mjpeg')
        end
        print('img1', img1, size1)
--        local ud = carray.byte(img1, size1)
--        file1:write(tostring(ud))
--        file1:close()
--        error()
    end
--    if (img2 ~= -1) then
--        print('img2', img2, size2)
--    end
--    if (img3 ~= -1) then
--        print('img3', img3)
--    end
--    if (img4 ~= -1) then
--        print('img4', img4)
--    end
--    if (img5 ~= -1) then
--        print('img5', img5)
--    end 
end
--]]
