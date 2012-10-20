----------------------------------------------------------------------
-- filter : FIR/IIR digital filter implementation
----------------------------------------------------------------------

require('vector')

filter = {}
filter.__index = filter

function filter.new(b, a)
  o = {}
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
  return filter.new({Ts/2, Ts/2}, {1, -1})
end

function filter.new_differentiator(Ts, w)
  local w = w or math.huge
  filter.new({2*w, -2*w}, {w*Ts + 2, w*Ts - 2})
end

function filter.new_first_order(Ts, w)
  filter.new({w*Ts, w*Ts}, {w*Ts + 2, w*Ts - 2})
end

function filter.new_second_order(Ts, w, Q)
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

function filter:set_output_limits(min_output, max_output)
  self.min_output = min_output
  self.max_output = max_output
end

function filter:reset()
  self.input = {}
  self.output = {}
end

function filter:update(input)
  -- update input / output history
  for i = #self.b, 1, -1 do
    self.input[i] = self.input[i-1] or input
  end
  for i = #self.a, 1, -1 do
    self.output[i] = self.output[i-1] or 0
  end
  -- return linear filter output
  for i = 1, #self.b do
    self.output[1] = self.output[1] + self.b[i]*self.input[i]
  end
  for i = 2, #self.a do
    self.output[1] = self.output[1] - self.a[i]*self.output[i]
  end
  self.output[1] = math.max(self.output[1], self.min_output)
  self.output[1] = math.min(self.output[1], self.max_output)
  return self.output[1]
end

return filter
