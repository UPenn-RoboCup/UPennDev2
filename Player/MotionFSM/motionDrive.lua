local state = {}
state._NAME = ...

--MotionDrive
--Stiffen all the legs, and step on the gas pedal when called

require'mcm'
require'hcm'
local Body       = require'Body'
local K          = Body.Kinematics
local util       = require'util'
local vector     = require'vector'
local timeout    = 20.0
local t_readings = 0.20
local t_settle   = 0.10
-- Declare local so as not to pollute the global namespace
-- This is 5.1 and 5.2 compatible
-- NOTE: http://www.luafaq.org/#T1.37.1
local t_entry, t_update, t_finish


--local dqLegLimit = vector.new{10,10,10,10,10,10}*Body.DEG_TO_RAD

dpMaxDelta = vector.new{.04, .03, .03, .4, .1, .4}



local pTorso, qLLeg, qRLeg

stage = 1

local qRLeg0
local qRLegTarget
local qRLegCurrent
local t_gas_timeout = 0

--local pedal_time = 2.0
local pedal_time = 1.5

function state.entry()
  print(state._NAME..' Entry' )

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t_entry
  
  --SJ: Now we only use commanded positions
  --As the actual values are read at motionIdle state
  qLLeg = Body.get_lleg_command_position()
  qRLeg = Body.get_rleg_command_position()

  --P gain for leg and ankle
  local pg = 16
  local ag = 8

  for i=1,10 do
    Body.set_rleg_position_p({pg,pg,pg,pg,pg,ag})
    unix.usleep(1e6*0.01);

    Body.set_lleg_position_p({pg,pg,pg,pg,pg,ag})
    unix.usleep(1e6*0.01);
  end

  qRLeg0 = qRLeg --Store information
  qRLegTarget = Body.get_rleg_command_position()
  qRLegCurrent = Body.get_rleg_command_position()
  local t_gas_timeout = 0


  hcm.set_drive_pedal_ankle_pitch(20*Body.DEG_TO_RAD)
  hcm.set_drive_pedal_knee_pitch(0*Body.DEG_TO_RAD)
end

---
--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()
  
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  if t_gas_timeout>0 then
    if t>t_gas_timeout then
      t_gas_timeout = 0
      qRLegTarget = qRLeg0
    end
  elseif hcm.get_drive_gas_pedal()>0 then
    t_gas_timeout = t + pedal_time
    qRLegTarget = {
      qRLeg0[1],
      qRLeg0[2],
      qRLeg0[3],
      qRLeg0[4]+hcm.get_drive_pedal_knee_pitch(),
      qRLeg0[5]+hcm.get_drive_pedal_ankle_pitch(),
      qRLeg0[6],
    }
    hcm.set_drive_gas_pedal(0)
  end
  
  qRLegCurrent = util.approachTolRad(qRLegCurrent, qRLegTarget,
    vector.new({1,1,1,1,1,1})*45*math.pi/180,dt)
  Body.set_rleg_command_position(qRLegCurrent)

end

function state.exit()
  print(state._NAME..' Exit.  Time elapsed:',t_finish-t_entry )

end

return state
