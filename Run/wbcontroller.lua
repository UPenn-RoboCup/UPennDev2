local cwd = os.getenv('PWD')
dofile(cwd.."/Player/include.lua")

local unix = require 'unix'

while true do
  print(unix.time())
end
