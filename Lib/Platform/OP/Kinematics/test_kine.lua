K = require 'OPKinematics'

a_u = .06;
a_l = .129

angles = K.inverse_arm( {0, 1*(a_u+a_l),0} );
pos = K.forward_rarm({0, math.pi/2, 0});

--[[
print('Pos\n===')
print(unpack(pos[1]))
print(unpack(pos[2]))
print(unpack(pos[3]))
print()
--]]
print()
print('Angles: ', unpack(angles) )
