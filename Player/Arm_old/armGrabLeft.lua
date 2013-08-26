module(..., package.seeall);

require('unix')
Config = require('ConfigPenn')
Body = require(Config.Body);

qLArmInit=Config.arm.qLArmInit;
qRArmInit=Config.arm.qRArmInit;

targetpos = {0.30,0.0,0.10};
t_grip = 3.0;
t_wait = 1.0;


function set_targetpos(pos)
  targetpos = pos;
end

function entry()
  print(_NAME..' Entry' ) 
  Body.enable_larm_linear_movement(false); 
  qLArm = Body.get_larm_command_position();
  qRArm = Body.get_rarm_command_position();
  trLArm = Kinematics.l_arm_torso(qLArm); 
  trRArm = Kinematics.r_arm_torso(qRArm); 
    
  posL0 = {trLArm[1],trLArm[2],trLArm[3]};
  posR0 = {trRArm[1],trRArm[2],trRArm[3]};
  
  phase = 1;
  t0 = unix.time();
  
  distR = math.sqrt(  (posR0[1]-targetpos[1])^2+
          (posR0[2]-targetpos[2])^2+
          (posR0[3]-targetpos[3])^2 );
          
  t_grip = distR/0.03; --3cm per sec        
end


function update()
--  print(_NAME..' Update' ) 
  t=unix.time();
    
  if phase==2 then
    ph = (t-t0)/t_wait;
    if ph>1 then
      Body.set_lhand_position(0.8);    
      return "done";
    end 
    return;
  end
  
  ph = (t-t0)/t_grip;
  if (ph>1) then 
    phase = phase + 1;
    t0 = unix.time();
    return;  
  end
  qLArm = Body.get_larm_command_position();
  qRArm = Body.get_rarm_command_position();

  trLArm[1] = posL0[1] * (1-ph) + targetpos[1] * ph;
  trLArm[2] = posL0[2] * (1-ph) + targetpos[2] * ph;
  trLArm[3] = posL0[3] * (1-ph) + targetpos[3] * ph;

  trRArm[1] = posR0[1] * (1-ph) + targetpos[1] * ph;
  trRArm[2] = posR0[2] * (1-ph) + targetpos[2] * ph;
  trRArm[3] = posR0[3] * (1-ph) + targetpos[3] * ph;

  
  qLInv = Kinematics.inverse_l_arm(trLArm, qLArm);
  qRInv = Kinematics.inverse_r_arm(trRArm, qRArm);
  
  checkLeft,checkLeftR = Body.check_larm_ik(trLArm);
  checkRight,checkRightR = Body.check_rarm_ik(trRArm);

--[[      
  if checkRight<0.01 and checkRightR<1*math.pi/180 then
    qRInv = Kinematics.inverse_r_arm(trRArm, qRArm);
    Body.set_rarm_target_position(qRInv);   
  end
--]]


  if checkLeft<0.01 and checkLeftR<1*math.pi/180 then
    qLInv = Kinematics.inverse_l_arm(trLArm, qLArm);
    Body.set_larm_target_position(qLInv);   
  end
  
end

function exit()
  print(_NAME..' Exit' ) 

end
