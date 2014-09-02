import sympybotics

print('Define the Robot')

dh_params = [('-pi/2', 0, 0, 'q'),
('pi/2', 0, 0, 'pi/2+q'),
('pi/2', 0, 0.246, 'pi/2+q'),
('pi/2', 0.030, 0, 'q'),
('-pi/2', -0.030, 0.250, '-pi/2+q'),
('-pi/2', 0, 0, 'q'),
('pi/2', 0, 0, 'q'),
('-pi/2', 0, 0, '-pi/2')]

rbtdef = sympybotics.RobotDef('THOR-OP 7DOF Arm', dh_params, dh_convention='standard')

rbtdef.dynparms()

rbt = sympybotics.RobotDynCode(rbtdef, verbose=True)

print('Generate the C code for the inverse dynamics')
tau_str = sympybotics.robotcodegen.robot_code_to_func('C', rbt.invdyn_code, 'tau_out', 'tau', rbtdef)

print('Find the base parameters for dynamics')
rbt.calc_base_parms()

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
    
f = open("inverse_dynamics.txt", "w")
try:
    f.write(str(rbt.dyn.baseparms))
    f.write('\n')
    f.write(tau_str)
finally:
    f.close()
    
f = open("DH Parameters used.txt", "w")
try:
    f.write('(alpha, a, d, theta)\n')
    f.write(str(dh_params))
finally:
    f.close()