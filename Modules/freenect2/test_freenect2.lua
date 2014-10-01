dofile'../../include.lua'
require'freenect2'
serial_number, firmware_version = freenect2.init()
print("serial_number, firmware_version:", serial_number, firmware_version)
for i=1,6 do
	print('\nUpdate', i)
	rgb, depth, ir = freenect2.update()
end

-- Show the data
print('Process the data', rgb)
print()
print('RGB')
for k, v in pairs(rgb) do
	if k~='data' then print(k,v) else print(k, #v) end
end

print()
print('Depth')
for k, v in pairs(depth) do print(k,v) end

print()
print('IR')
for k, v in pairs(ir) do print(k,v) end

print('JPEG save the rgb')
local c_rgb = require'jpeg'.compressor('rgb')
c_rgb:quality(95)
print('Compressing...')
img_jpeg = c_rgb:compress(rgb.data, rgb.width, rgb.height)
print('Compressed', #img_jpeg)
f = io.open('test.jpeg', 'w')
f:write(img_jpeg)
f:close()

print('Shutting down')
freenect2.shutdown()
