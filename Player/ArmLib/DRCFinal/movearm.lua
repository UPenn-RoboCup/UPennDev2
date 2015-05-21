local movearm={}
local Body   = require'Body'
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'
require'hcm'


mcm.set_arm_dpVelLeft(Config.arm.vel_linear_limit)
mcm.set_arm_dpVelRight(Config.arm.vel_linear_limit)
mcm.set_arm_dqVelLeft(Config.arm.vel_angular_limit)
mcm.set_arm_dqVelRight(Config.arm.vel_angular_limit)


function movearm.setArmJoints(qLArmTarget,qRArmTarget, dt,dqArmLim, absolute)
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()  

  local dqVelLeft = mcm.get_arm_dqVelLeft()
  local dqVelRight = mcm.get_arm_dqVelRight()

  local qL_approach, doneL2 = util.approachTolRad( qLArm, qLArmTarget, dqVelLeft, dt )  
  local qR_approach, doneR2 = util.approachTolRad( qRArm, qRArmTarget, dqVelRight, dt )
  
  --Dynamixel is STUPID so we should manually check for the direction
  --SJ: this may induce arm corkscrewing - should check 360 degree jump and just solve it instead

  if not absolute then  
    for i=1,7 do
      local qL_increment = util.mod_angle(qL_approach[i]-qLArm[i])
      local qR_increment = util.mod_angle(qR_approach[i]-qRArm[i])
      qL_approach[i] = qLArm[i] + qL_increment
      qR_approach[i] = qRArm[i] + qR_increment
    end
  end

  Body.set_larm_command_position( qL_approach )
  Body.set_rarm_command_position( qR_approach )
  if doneL2 and doneR2 then return 1 end
end


return movearm
