local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()

--Door opening state using HOOK
local handle_clearance = vector.new({0,0,-0.05})
local lhand_rpy0 = {90*Body.DEG_TO_RAD,0,0}
local rhand_rpy0 = {-90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0}
local trLArm1, trRArm1
local stage

local qLArmInit0,qRArmInit0
local qLArm0, qRArm0, qLArmComp, qRArmComp
local qWaist


local uTorsoComp0

local comactual

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  qLArm0 = mcm.get_arm_qlarm()
  qRArm0 = mcm.get_arm_qrarm()

  qLArmComp = mcm.get_arm_qlarm()
  qRArmComp = mcm.get_arm_qrarm()

  --[[
  qLArm0 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})  
  qRArm0 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  --]]
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

  hcm.set_hands_left_tr(trLArm0)
  hcm.set_hands_right_tr(trRArm0)
  hcm.set_hands_left_tr_target(trLArm0)
  hcm.set_hands_right_tr_target(trRArm0)

  local qWaist = Body.get_waist_command_position()
  local com = Kinematics.com_upperbody(qWaist,qLArm0,qRArm0,
        mcm.get_stance_bodyTilt(), 0,0)        
  uTorsoComp0 = {-com[1]/com[4],-com[2]/com[4]}

print("bodyTilt:",mcm.get_stance_bodyTilt())
print("qLArm:",unpack(qLArm0))
print("UTorsoComp0:",unpack(uTorsoComp0))
print("armplanner:",unpack(mcm.get_stance_uTorsoCompBias() ) )

--  arm_planner:reset_torso_comp(qLArm0, qRArm0)
--  arm_planner:save_boundary_condition({qLArm0, qRArm0, qLArm0, qRArm0, {0,0}})


  stage = "wristturn";  
end


function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  if plan_failed then return "planfail" end
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  
  if stage=="wristturn" then --Turn yaw angles first
    if movearm.setArmJoints(qLArm0,qRArm0,dt, Config.arm.slow_limit) ==1 then       
      stage = "teleopwait"      
    end
  elseif stage=="teleopwait" then       
        
    if hcm.get_state_proceed()==1 then
      qWaist = Body.get_waist_command_position()
      local qWaist, done1 = util.approachTol(
        qWaist,

--        vector.new({0,79*Body.DEG_TO_RAD}),
        vector.new({0,19*Body.DEG_TO_RAD}),
--        vector.new({0,79*Body.DEG_TO_RAD}),

        vector.new({0,10*Body.DEG_TO_RAD}),
        dt)
      Body.set_waist_command_position(qWaist)

      --New qArm0 considering rotated waist
      local qLArmNew = Body.get_inverse_larm(qLArm0,trLArm0)
      local qRArmNew = Body.get_inverse_rarm(qRArm0,trRArm0)

      local com = Kinematics.com_upperbody(qWaist,qLArmNew,qRArmNew,mcm.get_stance_bodyTilt(), 0,0)        
      local uTorsoComp = {-com[1]/com[4],-com[2]/com[4]}      
      local uTorsoCompX = uTorsoComp[1]-uTorsoComp0[1]
      local uTorsoCompY = uTorsoComp[2]-uTorsoComp0[2]
      mcm.set_stance_uTorsoComp({uTorsoCompX, uTorsoCompY})
      qLArmComp = Body.get_inverse_larm(qLArm0,trLArm0+ vector.new({-uTorsoCompX, -uTorsoCompY,0, 0,0,0}),
        qLArm0[3], mcm.get_stance_bodyTilt(), qWaist)
      qRArmComp = Body.get_inverse_rarm(qRArm0,trRArm0+ vector.new({-uTorsoCompX, -uTorsoCompY,0, 0,0,0}),
          qRArm0[3], mcm.get_stance_bodyTilt(), qWaist)
      local done2 = movearm.setArmJoints(qLArmComp,qRArmComp,dt)
      arm_planner:save_boundary_condition({qLArmNew, qRArmNew, qLArmComp, qRArmComp, 
        {uTorsoCompX,uTorsoCompY}})


    elseif hcm.get_state_proceed()==-1 then
      qWaist = Body.get_waist_command_position()
      local qWaist = util.approachTol(
        qWaist,
--        vector.new({-20*Body.DEG_TO_RAD,0}),
--        vector.new({1*B=dy.DEG_TO_RAD,0}),
        vector.new({0,0*Body.DEG_TO_RAD}),
        vector.new({0,5*Body.DEG_TO_RAD}),        
        dt)
      Body.set_waist_command_position(qWaist)

      local com = Kinematics.com_upperbody(qWaist,qLArm,qRArm,mcm.get_stance_bodyTilt(), 0,0)        
      local uTorsoComp = {-com[1]/com[4],-com[2]/com[4]}
      local uTorsoCompX = uTorsoComp[1]-uTorsoComp0[1]
      local uTorsoCompY = uTorsoComp[2]-uTorsoComp0[2]
      mcm.set_stance_uTorsoComp({uTorsoCompX, uTorsoCompY})

      qLArmComp = Body.get_inverse_larm(qLArm,trLArm0+ vector.new({-uTorsoCompX, -uTorsoCompY,0, 0,0,0}) )
      qRArmComp = Body.get_inverse_rarm(qRArm,trRArm0+ vector.new({-uTorsoCompX, -uTorsoCompY,0, 0,0,0}) )
       
      local done2 = movearm.setArmJoints(qLArmComp,qRArmComp,dt)
comactual= {com[1]/com[4],com[2]/com[4]}
    end
  elseif stage=="teleopmove" then 
    if arm_planner:play_arm_sequence(t) then 
      stage="teleopwait"
    end
  end
  --hcm.set_state_proceed(0)
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
