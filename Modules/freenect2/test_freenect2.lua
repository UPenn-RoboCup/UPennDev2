require'freenect2'
serial_number, firmware_version = freenect2.init()
print("serial_number, firmware_version:", serial_number, firmware_version)

local meta = {
	serial_number = serial_number,
	firmware_version = firmware_version,
}

-- Log data with libLog
for i=1,120 do
	rgb, depth, ir = freenect2.update()
	print(i)
end

print('Shutting down')
freenect2.shutdown()
