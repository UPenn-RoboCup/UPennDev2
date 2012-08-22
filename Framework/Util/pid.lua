module(..., package.seeall)

--basic pid controller

local mt = {}

local function max(a, b)
  return a > b and a or b
end

local function min(a, b)
  return a < b and a or b
end

function new(pgain, igain, dgain)
  o = {}
  -- gains
  o.pgain = pgain or 0
  o.igain = igain or 0
  o.dgain = dgain or 0
  -- filter parameters
  o.ilimit = math.huge
  o.dT = 0
  -- book-keeping
  o.p = 0 -- error 
  o.i = 0 -- integral
  o.d = 0 -- derivative
  return setmetatable(o, mt)
end

function set_gains(o, pgain, igain, dgain)
  o.pgain = pgain
  o.igain = igain
  o.dgain = dgain
end

function set_integral_limit(o, ilimit)
  o.ilimit = ilimit 
end

function set_filter_constant(o, dT)
  o.dT = dT
end

function reset(o)
  -- clear pid data
  o.p = 0
  o.i = 0
  o.d = 0
  o.process_value = nil
end

function update(o, set_point, process_value, dt)
--estimate proportional, integral and derivative terms
  local e = set_point - process_value
  o.process_value = o.process_value or process_value
  o.p = e
  o.i = max(min(o.i + e*dt, o.ilimit), -o.ilimit)
  o.d = (o.dT*o.d - (process_value - o.process_value))/(o.dT + dt + 1e-12)
  o.process_value = process_value
--return control output
  return (o.pgain*o.p + o.igain*o.i + o.dgain*o.d)
end

mt.__index = {
  set_gains = set_gains,
  set_integral_limit = set_integral_limit,
  set_filter_constant = set_filter_constant,
  reset = reset,
  update = update,
}
