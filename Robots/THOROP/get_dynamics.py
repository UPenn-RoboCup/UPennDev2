import sympybotics

print('Define the Robot')

'''
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, PI/2+q[2], upperArmLength)
    .mDH(PI/2, elbowOffsetX, q[3], 0)
    .mDH(-PI/2, -elbowOffsetX, -PI/2+q[4], lowerArmLength)
    .mDH(-PI/2, 0, q[5], 0)
    .mDH(PI/2, 0, q[6], 0)
    .mDH(-PI/2, 0, -PI/2, 0)
'''

dh_params = [
('-pi/2', 0, 0, 'q'),
('pi/2', 0, 0, 'q+pi/2'),
('pi/2', 0, 'upperArmLength', 'q+pi/2'),
('pi/2', 'elbowOffsetX', 0, 'q'),
('-pi/2', '-elbowOffsetX', 'lowerArmLength', 'q-pi/2'),
('-pi/2', 0, 0, 'q'),
('pi/2', 0, 0, 'q'),
('-pi/2', 0, 0, '-pi/2')
]

#rbtdef = sympybotics.RobotDef('THOR-OP 7DOF Arm', dh_params, dh_convention='standard')
rbtdef = sympybotics.RobotDef('THOR-OP 7DOF Arm', dh_params, dh_convention='modified')

dp = rbtdef.dynparms()

rbt = sympybotics.RobotDynCode(rbtdef, verbose=True)

print('Generate the C code for the inverse dynamics')
tau_str = sympybotics.robotcodegen.robot_code_to_func('C', rbt.invdyn_code, 'tau_out', 'tau', rbtdef)

# Just printing stuff
#rbt.geo.T[-1]
#rbt.kin.J[-1]
#print(tau_str)
#rbt.dyn.baseparms
#dh_params

# Save stuff

f = open("jacobian.txt", "w")
try:
    f.write(str(rbt.kin.J[-1]))
finally:
    f.close()

f = open("jacobian_com.txt", "w")
try:
    f.write(str(rbt.kin.Jc[-1]))
finally:
    f.close()

f = open("fk.txt", "w")
try:
    f.write(str(rbt.geo.T[-1]))
finally:
    f.close()

f = open("DH Parameters used.txt", "w")
try:
    f.write('(alpha, a, d, theta)\n')
    f.write(str(dh_params))
finally:
    f.close()

f = open("Synamic Barycentric Parameters.txt", "w")
try:
    f.write(str(dp))
finally:
    f.close()

#print('Find the base parameters for dynamics')
#rbt.calc_base_parms(verbose=True)
#f = open("inverse_dynamics.txt", "w")
#try:
#    f.write(str(rbt.dyn.baseparms))
#    f.write('\n')
#    f.write(tau_str)
#finally:
#    f.close()
