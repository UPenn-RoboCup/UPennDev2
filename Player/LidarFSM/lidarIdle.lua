local Body = require'Body'

local lidarIdle = {}
lidarIdle._NAME = 'lidarIdle'

function lidarIdle.entry()
  print(_NAME..' Entry' ) 
end

function lidarIdle.update()
--  print(_NAME..' Update' )
-- TODO: Torque off?
end

function lidarIdle.exit()
  print(_NAME..' Exit' ) 
end

return lidarIdle