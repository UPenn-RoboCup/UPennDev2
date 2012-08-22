require('Kinematics')

q = {0,-0.1,0.05,0.1,0.2,0.5,-0.1,0.2,0.3,0.1,0.2,0.1}
r = Kinematics.inverse_joints(q)

r[2]=0.041
r[3]=r[2]
r[4]=0.073
r[5]=0.038
r[6]=0.038

for k,v in pairs(r) do print(k,v) end
print('\n')

q2 = Kinematics.forward_joints(r)

for k,v in pairs(q2) do print(k,v) end
