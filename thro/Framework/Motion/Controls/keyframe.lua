require('vector')
require('trajectory')

----------------------------------------------------------------------
-- keyframe : keyframe motion player
----------------------------------------------------------------------

keyframe = {}
keyframe.__index = keyframe
keyframe.__mtstring = 'keyframe'

function keyframe.new(page)
  local o = {}
  o.name = page.name
  o.steps = page.steps
  o.active = false
  o.t_step = {0}
  o.q_spline = {}
  keyframe.initialize(o, o.steps[1].joint_position)
  return setmetatable(o, keyframe)
end

function keyframe.new_page(_name, _steps)
  return {
    name = _name or '',
    steps = _steps or {}
  }
end

function keyframe.new_step(_joint_position, _duration, _pause)
  return {
    joint_position = _joint_position,
    duration = _duration or 2.5,
    pause = _pause or 0
  }
end

function keyframe.get_page(o)
  return keyframe.new_page(o.name, o.steps) 
end

function keyframe.get_name(o)
  return o.name
end

function keyframe.get_step(o, i)
  return o.steps[i]
end

function keyframe.get_duration(o)
  return o.t_step[#o.t_step]
end

function keyframe.initialize(o, q0)
  -- compute keyframe trajectory given initial joint positions q0
  local q_past = q0
  local m_past = vector.zeros(#q0)
  assert(#q0 == #o.steps[1].joint_position)

  -- compute piecewise hermite curves for each keyframe step
  for i = 1, #o.steps do
    local step = o.steps[i]
    local nextstep = o.steps[i+1] or step
    local q_step = vector.new(step.joint_position)
    local m_step = vector.zeros(#step.joint_position)
    if (step.pause == 0 and nextstep ~= step) then
      for j = 1,#step.joint_position do
	m_past[j] = (step.joint_position[j] - q_past[j])/step.duration
	m_step[j] = (nextstep.joint_position[j] - q_step[j])/nextstep.duration
	-- prevent overshoot scenarios
	if (i == 1) or (m_past[j] == 0) or (m_step[j]*m_past[j] < 0) then
	  m_step[j] = 0
	end
      end
    end
    o.q_spline[i] = trajectory.hermite_curve(
      {q_past, m_past},
      {q_step, m_step},
      step.duration
    )
    o.t_step[i+1] = o.t_step[i] + step.duration + step.pause
    q_past = q_step
    m_past = m_step
  end
end

function keyframe.evaluate(o, t)
  -- return joint configuration at time t
  for i = 1, #o.steps do
    if (t < o.t_step[i+1]) then
      return o.q_spline[i](t - o.t_step[i])
    end
  end
  return o.steps[#o.steps].joint_position
end

return keyframe
