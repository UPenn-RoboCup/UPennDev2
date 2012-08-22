----------------------------------------------------------------------
-- keyframe : plays keyframe actions
----------------------------------------------------------------------

require('Body')
require('unix')
require('vector')
require('Config')
require('interpol')
require('MotionState') 

-- Setup 
----------------------------------------------------------------------

keyframe = MotionState.new(...)
local sensor = keyframe.sensor
local actuator = keyframe.actuator

local action_table = {}
local action_index = {}
local action_queue = {}
local joint = Config.joint

-- book-keeping
local steps = {}
local iStep = 1
local mStep = 0 
local qStep = sensor:get_joint_position() 
local qSpline = nil
local t0 = Body.get_time()

-- Utilities
----------------------------------------------------------------------

local function update_step_parameters()
  local step = steps[iStep]
  local nextstep = steps[iStep + 1] or step
  local qPast = qStep
  local mPast = mStep
  if (not step) then return end
  -- get joint angles for current step
  qStep = vector.new(step.joint_position)
  -- get joint trajectory slopes for current step
  mStep = vector.zeros(#joint.id)
  if (step.pause == 0 and steps[iStep + 1]) then
    for i = 1,#joint.id do
      -- using the three point difference method
      mStep[i] = (step.joint_position[i] - qPast[i])/(2*step.duration)
        + (nextstep.joint_position[i] - qStep[i])/(2*nextstep.duration)
    end
  end
  -- get spline function for the current step interval
  qSpline = interpol.hermite_curve(qPast, qStep, mPast, mStep, 0, step.duration)
  -- initialize time
  t0 = Body.get_time()
end

-- Interface
----------------------------------------------------------------------

function keyframe:load_action_table(actions)
  -- load a keyframe motion table
  action_table = actions
  action_queue = {}
  action_index = {}
  for i = 1,#action_table do 
    action_index[action_table[i].name] = i
  end
  steps = {}
end

function keyframe:play(action)
  -- add an action to the play queue
  if (type(action) == 'number') then
    table.insert(action_queue, action_table[action])
  elseif (type(action) == 'string') then
    table.insert(action_queue, action_table[action_index[action]])
  elseif (type(action) == 'table') then
    table.insert(action_queue, action)
  else
    return false
  end
  return true
end

function keyframe:stop()
  action_queue = {}
  steps = {}
end

function keyframe:entry()
  self:set_joint_access(1, 'all')
  keyframe.active = false
  steps = {}
end

function keyframe:update()
  if steps[iStep] then
    keyframe.active = true
    -- update joint trajectories
    local step = steps[iStep]
    local t = Body.get_time()
    local q = qSpline(t - t0)
    actuator:set_joint_position(q)
    -- advance to next keyframe step
    if ((t - t0) > (step.duration + step.pause)) then
      iStep = iStep + 1
      update_step_parameters()
    end
  else
    if (#action_queue > 0) then
      -- load new keyframe action
      steps = table.remove(action_queue, 1).steps
      iStep = 1
      qStep = vector.new(sensor:get_joint_position()) 
      mStep = vector.zeros(#joint.id)
      update_step_parameters()
    else
      -- exit
      keyframe.active = false
      return 'done'
    end
  end
end

function keyframe:exit()
  keyframe.active = false
end

return keyframe
