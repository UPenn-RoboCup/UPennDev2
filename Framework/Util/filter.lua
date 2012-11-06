----------------------------------------------------------------------
-- filter : FIR/IIR digital filter implementation
----------------------------------------------------------------------

require('vector')
require('math')

filter = {}
filter.__index = filter

function filter.new(b, a)
  -- b, a : LTI difference equation coefficients 
  local o = {}
  o.b = b or {1}
  o.a = a or {1}
  o.input = {}
  o.output = {}
  o.min_output = -math.huge
  o.max_output = math.huge
  for i = #o.b, 1, -1 do
    o.b[i] = o.b[i]/o.a[1]
  end
  for i = #o.a, 1, -1 do
    o.a[i] = o.a[i]/o.a[1]
  end
  return setmetatable(o, filter)
end

function filter.new_integrator(Ts)
  -- Ts : time step
  return filter.new({Ts/2, Ts/2}, {1, -1})
end

function filter.new_differentiator(Ts, f)
  -- Ts : time step
  -- f  : corner frequency
  local w = 2*math.pi*f
  return filter.new({2*w, -2*w}, {w*Ts + 2, w*Ts - 2})
end

function filter.new_low_pass(Ts, f)
  -- Ts : time step
  -- f  : corner frequency
  local w = 2*math.pi*f
  return filter.new({w*Ts, w*Ts}, {w*Ts + 2, w*Ts - 2})
end

function filter.new_second_order_differentiator(Ts, f, Q)
  -- Ts : time step
  -- f  : corner frequency
  -- Q  : Q factor
  local w = 2*math.pi*f
  local Q = Q or 1/2
  local b = {}
  b[1] = 2*w^2*Ts
  b[2] = 0
  b[3] = -2*w^2*Ts
  local a = {}
  a[1] = w^2*Ts^2 + 2*w*Ts/Q + 4
  a[2] = 2*w^2*Ts^2 - 8
  a[3] = w^2*Ts^2 - 2*w*Ts/Q + 4
  return filter.new(b, a)
end

function filter.new_second_order_low_pass(Ts, f, Q)
  -- Ts : time step
  -- f  : corner frequency
  -- Q  : Q factor
  local w = 2*math.pi*f
  local Q = Q or 1/2
  local b = {}
  b[1] = w^2*Ts^2
  b[2] = 2*w^2*Ts^2
  b[3] = w^2*Ts^2
  local a = {}
  a[1] = w^2*Ts^2 + 2*w*Ts/Q + 4
  a[2] = 2*w^2*Ts^2 - 8
  a[3] = w^2*Ts^2 - 2*w*Ts/Q + 4
  return filter.new(b, a)
end

function filter.set_output_limits(o, min_output, max_output)
  o.min_output = min_output
  o.max_output = max_output
end

function filter.reset(o)
  o.input = {}
  o.output = {}
end

function filter.update(o, input)
  -- update input / output history
  for i = #o.b, 1, -1 do
    o.input[i] = o.input[i-1] or input
  end
  for i = #o.a, 1, -1 do
    o.output[i] = o.output[i-1] or 0
  end
  -- return filter output
  for i = 1, #o.b do
    o.output[1] = o.output[1] + o.b[i]*o.input[i]
  end
  for i = 2, #o.a do
    o.output[1] = o.output[1] - o.a[i]*o.output[i]
  end
  o.output[1] = math.max(o.output[1], o.min_output)
  o.output[1] = math.min(o.output[1], o.max_output)
  return o.output[1]
end

function filter.__tostring(o)
  local str = '' 
  for k,v in pairs(o) do
    str = str..string.format('\n%20s : %s', k, tostring(v))
  end
  return str..'\n'
end

return filter
