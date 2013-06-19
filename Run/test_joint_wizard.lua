dofile'include.lua'
local jcm = require'jcm'
local positions = jcm:get_position()
for i,v in ipairs(positions) do
print(i,'at',v)
end
positions[17] = 90
jcm:set_command_position( positions )
