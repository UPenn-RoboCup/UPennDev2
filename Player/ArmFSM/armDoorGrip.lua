local state = {}
state._NAME = 'armDoorGrip'
local Config = require'Config'
local Body   = require'Body'
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'
require'hcm'

-- Angular velocity limit
local dqArmMax = vector.new({10,10,10,15,45,45})*Body.DEG_TO_RAD

local turnAngle = 0
local function calculate_arm_position()
  local body_pos   = vector.new{0,0,0}
  local body_rpy   = vector.new{0,0,0}
  local handle     = hcm.get_door_handle()
  local open_angle = hcm.get_door_open_ang()

  local trHandle = T.eye()
      * T.trans(unpack(vector.slice(handle,1,3)) )
      * T.rotZ(open_angle)

  local trGrip = trHandle
      * T.rotX(handle[4])
      * T.trans(0,handle[4]/2,0)
       
  local trBody = T.eye()
      * T.trans( unpack(body_pos) )
      * T.rotZ(body_rpy[3])
      * T.rotY(body_rpy[2])
  	   
  local trArm = T.position6D(T.inv(trBody)*trGrip)
  return trArm
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  
  -- Calculate where we need to place our arm  
  local trArm = calculate_arm_position()
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local qLInv = Kinematics.inverse_l_arm(trArm, qLArm)
  local qRInv = Kinematics.inverse_r_arm(trArm, qRArm)

  -- Go to the target arm position
  qLArm = util.approachTol( qLArm, qLInv, dqArmMax, dt )
  if qLArm~=true then Body.set_larm_command_position( qLArm ) end
  qRArm = util.approachTol( qRArm, qRInv, dqArmMax, dt )
  if qRArm~=true then Body.set_rarm_command_position( qRArm ) end

  if qLArm==true and qRArm==true then
    Body.set_lgrip_percent(.5)
    Body.set_rgrip_percent(.5)
    return'done'
  end
  
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state