require('Kinematics')
require('Transform')
require('vector')

Statics = {}

function Statics.get_com()
  -- return center of mass {x, y, z}
end

function Statics.get_cop()
  -- return center of pressure {x, y, z, roll, pitch, yaw}, magnitude
end

function Statics.get_l_foot_cop()
  -- return left foot center of pressure {x, y}, magnitude
end

function Statics.get_r_foot_cop()
  -- return right foot center of pressure {x, y}, magnitude
end

return Statics
