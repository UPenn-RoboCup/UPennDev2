import sympybotics

print('Define the Robot')

#[thorop] aThigh 0.099668652491162 5.7105931374996
#[thorop] aTibia 0.099668652491162 5.7105931374996
#[thorop] dThigh 0.30149626863363
#[thorop] dTibia 0.30149626863363

dh_params = [
(0, 0, 0, 'q+pi/2'),
('pi/2', 0, 0, 'q+pi/2'),
('pi/2', 0, 0, 'q+aThigh'),
('0', '-dThigh', 0, 'q-aTibia-aThigh'),
('0', '-dTibia', 0, 'q+aTibia'),
('-pi/2', 0, 0, 'q'),
]

#rbtdef = sympybotics.RobotDef('THOR-OP 7DOF Arm', dh_params, dh_convention='standard')
rbtdef = sympybotics.RobotDef('THOR-OP 6DOF Left Leg', dh_params, dh_convention='modified')

rbtdef.dynparms()

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

f = open("jacobian_lleg.txt", "w")
try:
    f.write(str(rbt.kin.J[-1]))
finally:
    f.close()
    
f = open("jacobian_com_lleg.txt", "w")
try:
    f.write(str(rbt.kin.Jc[-1]))
finally:
    f.close()
    
f = open("fk_lleg.txt", "w")
try:
    f.write(str(rbt.geo.T[-1]))
finally:
    f.close()
    
f = open("DH Parameters used lleg.txt", "w")
try:
    f.write('(alpha, a, d, theta)\n')
    f.write(str(dh_params))
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