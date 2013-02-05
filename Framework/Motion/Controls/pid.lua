require('filter')

---------------------------------------------------------------------------
-- pid : proportional integrator differentiator controller
---------------------------------------------------------------------------

pid = {}
pid.__index = pid
pid.__mtstring = 'pid'

local Q = 0.5

function pid.new(Ts, p_gain, i_gain, d_gain)
  local o = {}
  o.Ts = Ts or 1e-10
  o.p_gain = p_gain or 0
  o.i_gain = i_gain or 0
  o.d_gain = d_gain or 0
  o.min_output = -math.huge
  o.max_output = math.huge
  o.min_setpoint = -math.huge
  o.max_setpoint = math.huge
  o.setpoint = 0
  o.error = 0
  o.output = 0
  o.p_term = 0
  o.i_term = 0
  o.d_term = 0
  o.d_corner_frequency = 1/(2*o.Ts)
  o.d_filter = filter.new_second_order_differentiator(
    o.Ts, o.d_corner_frequency, Q)
  return setmetatable(o, pid)
end

function pid.set_output_limits(o, min_output, max_output)
  o.min_output = min_output
  o.max_output = max_output
end

function pid.set_setpoint_limits(o, min_setpoint, max_setpoint)
  o.min_setpoint = min_setpoint
  o.max_setpoint = max_setpoint
end

function pid.set_time_step(o, Ts)
  o.Ts = Ts
  o.d_corner_frequency = math.min(o.d_corner_frequency, 1/(2*o.Ts))
  o.d_filter = filter.new_second_order_differentiator(
    o.Ts, o.d_corner_frequency, Q)
end

function pid.set_gains(o, p_gain, i_gain, d_gain)
  o.p_gain = p_gain
  o.i_gain = i_gain
  o.d_gain = d_gain
  o.p_term = 0
  o.i_term = 0
  o.d_term = 0
end

function pid.set_p_gain(o, p_gain)
  o.p_gain = p_gain
  o.p_term = 0
end

function pid.set_i_gain(o, i_gain)
  o.i_gain = i_gain
  o.i_term = 0
end

function pid.set_d_gain(o, d_gain)
  o.d_gain = d_gain
  o.d_term = 0
end

function pid.set_d_corner_frequency(o, d_corner_frequency)
  o.d_corner_frequency = d_corner_frequency 
  o.d_corner_frequency = math.min(o.d_corner_frequency, 1/(2*o.Ts))
  o.d_filter = filter.new_second_order_differentiator(
    o.Ts, o.d_corner_frequency, Q)
end

function pid.set_setpoint(o, setpoint)
  o.setpoint = math.max(math.min(setpoint, o.max_setpoint), o.min_setpoint)
end

function pid.reset(o)
  o.p_term = 0
  o.i_term = 0
  o.d_term = 0
  o.error = 0
  o.d_filter:reset()
end

function pid.update(o, process_value)
  local e = o.setpoint - process_value
  local e_mean = (e + o.error)/2
  local d = o.d_filter:update(process_value)

  -- update pid terms
  local p_term = o.p_gain*e
  local i_term = o.i_gain*e_mean*o.Ts + o.i_term
  local d_term = o.d_gain*-d

  -- calculate output
  local output = p_term + i_term + d_term
  output = math.max(math.min(output, o.max_output), o.min_output)

  -- update control variables
  o.error = e
  o.output = output
  o.p_term = p_term
  o.d_term = d_term
  if ((output < o.max_output) and (output > o.min_output)) then
    o.i_term = i_term
  end

  return output
end

function pid.__tostring(o)
  local str = '' 
  for k,v in pairs(o) do
    if (k ~= 'd_filter') then
      str = str..string.format('\n%20s : %s', k, tostring(v))
    end
  end
  return str..'\n'
end

return pid
