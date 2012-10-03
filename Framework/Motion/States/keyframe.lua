----------------------------------------------------------------------
-- keyframe : plays keyframe motions
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
local dcm = keyframe.dcm

local keyframe_table = {}
local motion_index = {}
local motion_queue = {}
local joint = Config.joint

-- book-keeping
local steps = {}
local iStep = 1
local mStep = 0 
local qStep = dcm:get_joint_position_sensor() 
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

function keyframe:load_keyframe_table(motions)
  -- load a keyframe motion table
  keyframe_table = motions
  motion_queue = {}
  motion_index = {}
  for i = 1,#keyframe_table do 
    motion_index[keyframe_table[i].name] = i
  end
  steps = {}
end

function keyframe:play(motion)
  -- add a motion to the play queue
  if (type(motion) == 'number') then
    table.insert(motion_queue, keyframe_table[motion])
  elseif (type(motion) == 'string') then
    table.insert(motion_queue, keyframe_table[motion_index[motion]])
  elseif (type(motion) == 'table') then
    table.insert(motion_queue, motion)
  else
    return false
  end
  return true
end

function keyframe:stop()
  motion_queue = {}
  steps = {}
end

function keyframe:entry()
  self.running = true
  self:set_joint_access(1, 'all')
  steps = {}
  local q0 = dcm:get_joint_position_sensor('all')
  dcm:set_joint_force(0, 'all')
  dcm:set_joint_position(q0, 'all')
  dcm:set_joint_velocity(0, 'all')
  dcm:set_joint_stiffness(1, 'all')
  dcm:set_joint_damping(0, 'all')
end

function keyframe:update()
  if steps[iStep] then
    -- update joint trajectories
    local step = steps[iStep]
    local t = Body.get_time()
    local q = qSpline(t - t0)
    dcm:set_joint_position(q)
    -- advance to next keyframe step
    if ((t - t0) > (step.duration + step.pause)) then
      iStep = iStep + 1
      update_step_parameters()
    end
  else
    if (#motion_queue > 0) then
      -- load new keyframe motion
      steps = table.remove(motion_queue, 1).steps
      iStep = 1
      qStep = vector.new(dcm:get_joint_position_sensor()) 
      mStep = vector.zeros(#joint.id)
      update_step_parameters()
    else
      -- exit
      return 'done'
    end
  end
end

function keyframe:exit()
  self.running = false
end

return keyframe
