local cwd = os.getenv('PWD')
dofile(cwd.."/Player/include.lua")

local unix = require 'unix'
local Body = require 'Body'

print(Body.get_time())
--while true do
--  print(unix.time())
--end
