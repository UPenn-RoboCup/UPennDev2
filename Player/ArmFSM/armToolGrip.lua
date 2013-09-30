local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'

local dqArmMax = Config.arm.slow_limit
local dpArmMax = Config.arm.linear_slow_limit
local stage = 1;

local qLArm, qRArm, trLArm, trRArm, trTarget
local tool_pos,tool_pos_target


function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  --open gripper
  Body.set_lgrip_percent(0)
  Body.set_rgrip_percent(0)

  qLArm = Body.get_larm_command_position()
  qRArm = Body.get_rarm_command_position()
  trLArm = Body.get_forward_larm(qLArm);
  trRArm = Body.get_forward_rarm(qRArm)  

  tool_pos = hcm.get_tool_pos()
  tool_pos_target = hcm.get_tool_pos()


--tool_pos = {0.40,0.10,-0.05}

  stage = 1;  
end

function state.update()
  --  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  qLArm = Body.get_larm_command_position()
  qRArm = Body.get_rarm_command_position()
  trLArm = Body.get_forward_larm(qLArm)
  trRArm = Body.get_forward_rarm(qRArm)


--print(unpack(trLArm))
 

  if stage==1 then 
    trLArm[1],trLArm[2],trLArm[3] = tool_pos[1],tool_pos[2]+0.05,tool_pos[3]
    local ret = movearm.setArmToPositionAdapt(trLArm,trRArm,dt)    
    if ret==1 then stage=stage+1; end
  elseif stage==2 then
    trLArm[1],trLArm[2],trLArm[3] = tool_pos[1],tool_pos[2],tool_pos[3]
    local ret = movearm.setArmToPositionAdapt(trLArm,trRArm,dt)    
    if ret==1 then stage=stage+1; end
  else
    Body.set_lgrip_percent(1)

  --Read the pickup position from shm
  --TODO
  --[[
    --Now we can move the gripped tool around (teleop style)
    tool_approach = util.approachTol(tool_pos,tool_pos_target,{0.1,0.1,0.1},dt)
    trLArm[1],trLArm[2],trLArm[3] = tool_approach[1],tool_approach[2],tool_approach[3]
    local ret = movearm.setArmToPositionAdapt(trLArm,trRArm,dt)    
    if ret==-1 then --we cannot move to the direction, clear the hcm


    else
      tool_pos = tool_pos1
    end    
--]]      

  end
end

function state.exit()  
  Body.set_lgrip_percent(0)
  Body.set_rgrip_percent(0)
  print(state._NAME..' Exit' )
end

return state