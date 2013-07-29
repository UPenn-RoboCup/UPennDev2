dofile'include.lua'

local Body = require'Body'

Body.entry()

while true do
	print('Time:',Body.get_time())
	Body.update()
end
