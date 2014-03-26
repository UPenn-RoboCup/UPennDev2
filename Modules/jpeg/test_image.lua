local jpeg = require 'jpeg'
require 'unix'

local yuyv_filename = 'image_yuyv'
local yuyv_file = io.open(yuyv_filename, 'r')
local yuyv_str = yuyv_file:read('*a')

--local rgb_filename = 'image_rgb'
--local rgb_file = io.open(rgb_filename, 'r')
--local rgb_str = rgb_file:read('*a')

--for q=5,95,5 do
--	print("\nQuality",q)
--	jpeg.set_quality(q)
--	t0 = unix.time()
--	local yuyv_jpg = jpeg.compress_yuyv(yuyv_str, 640, 480)
--	t1 = unix.time()
--	local debug_yuyv = string.format("YUYV | Time: %.2f ms\tRatio: %2.2f%%\t%.2f kB",(t1-t0)*1000, 100*#yuyv_jpg/#yuyv_str, #yuyv_jpg/1024)
--
--	t0 = unix.time()
--	local rgb_jpg = jpeg.compress_rgb(rgb_str, 640, 480)
--	t1 = unix.time()
--	local debug_rgb = string.format("RGB  | Time: %.2f ms\tRatio: %2.2f%%\t%.2f kB",(t1-t0)*1000, 100*#rgb_jpg/#rgb_str, #rgb_jpg/1024)
--	
--	print(debug_yuyv)
--	print(debug_rgb)
--end
----[[
local c_yuyv = jpeg.compressor('yuyv')
c_yuyv:downsampling(0)
t0=unix.time()
for i=1,10 do c_yuyv:compress(yuyv_str, 640, 480) end
t1=unix.time()
local yuyv_jpg = c_yuyv:compress(yuyv_str, 640, 480)
--]]
--[[
t0=unix.time()
for i=1,10 do jpeg.compress_yuyv(yuyv_str, 640, 480) end
t1=unix.time()
local yuyv_jpg = jpeg.compress_yuyv(yuyv_str, 640, 480)
--]]
print("ok?",#yuyv_jpg,t1-t0)
local jpg_file = io.open('yuyv.jpg', 'w');
jpg_file:write(yuyv_jpg)
jpg_file:close()


c_yuyv:downsampling(0)
--c_yuyv:downsampling(1)
--c_yuyv:downsampling(2)
local yuyv_jpg_crop = c_yuyv:compress_crop(yuyv_str, 640, 480, 121, 321, 121, 240)
print("ok2?",#yuyv_jpg_crop)
local jpg_file = io.open('yuyv_crop.jpg', 'w');
jpg_file:write(yuyv_jpg_crop)
jpg_file:close()