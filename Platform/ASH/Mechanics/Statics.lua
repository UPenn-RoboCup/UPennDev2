require('Kinematics')
require('Transform')
require('vector')

Statics = {}

function Statics.get_CoG()
  -- return center of gravity {x, y, z}
end

function Statics.get_CoP()
  -- return center of pressure {x, y, z, roll, pitch, yaw}, magnitude
end

function Statics.get_l_foot_CoP()
  -- return left foot center of pressure {x, y}, magnitude
end

function Statics.get_r_foot_CoP()
  -- return right foot center of pressure {x, y}, magnitude
end

return Statics
