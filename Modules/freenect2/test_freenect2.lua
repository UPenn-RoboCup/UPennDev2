dofile'../../include.lua'
require'freenect2'
serial_number, firmware_version = freenect2.init()
print("serial_number, firmware_version:", serial_number, firmware_version)
for i=1,120 do
--	print('\nUpdate', i)
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
for k, v in pairs(depth) do
  if k~='data' then print(k,v) else print(k, #v) end
end

print()
print('IR')
for k, v in pairs(ir) do
  if k~='data' then print(k,v) else print(k, #v) end
end

print('JPEG save the rgb')
local c_rgb = require'jpeg'.compressor('rgb')
img_jpeg = c_rgb:compress(rgb.data, rgb.width, rgb.height)
f = io.open('rgb.jpeg', 'w')
f:write(img_jpeg)
f:close()
--
f = io.open('rgb.raw', 'w')
f:write(rgb.data)
f:close()

print('RAW save the IR')
f = io.open('ir.raw', 'w')
f:write(ir.data)
f:close()

print('RAW save the Depth')
f = io.open('depth.raw', 'w')
f:write(depth.data)
f:close()

print('Shutting down')
freenect2.shutdown()
