require'freenect2'
serial_number, firmware_version = freenect2.init()
print("serial_number, firmware_version:", serial_number, firmware_version)
--for i=1,6 do
	print('Update', i)
	freenect2.update()
--end
freenect2.shutdown()
