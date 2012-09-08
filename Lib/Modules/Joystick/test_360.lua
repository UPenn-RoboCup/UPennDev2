require 'xbox360'
require 'unix'

xbox360.open()

for i=1,100 do
	local buttons = xbox360.button()
	local axes = xbox360.axis()
	print('Buttons:', unpack(buttons) )
	print('Axes:', unpack(axes) )
	unix.usleep(1e5)
end

xbox360.close()