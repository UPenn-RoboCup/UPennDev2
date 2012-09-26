module(..., package.seeall)

--basic pid controller

local mt = {}

function new(pgain, igain, dgain)
  o = {}
  -- gains
  o.pgain = pgain or 0
  o.igain = igain or 0
  o.dgain = dgain or 0
  -- filter parameters
  o.ilimit = math.huge
  o.dconst = 0
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

function set_pgain(o, pgain)
  o.pgain = pgain
end

function set_igain(o, igain)
  o.igain = igain
end

function set_dgain(o, dgain)
  o.dgain = dgain
end

function set_ilimit(o, ilimit)
  -- set integral limit for anti-windup
  o.ilimit = ilimit 
end

function set_dconst(o, dconst)
  -- set time costant for derivative filter
  o.dconst = dconst
end

function reset(o)
  -- clear pid data
  o.p = 0
  o.i = 0
  o.d = 0
  o.process_value = nil
end

function update(o, setpoint, process_value, dt)
--estimate proportional, integral and derivative terms
  local e = setpoint - process_value
  o.process_value = o.process_value or process_value
  o.p = e
  o.i = math.max(math.min(o.i + e*dt, o.ilimit), -o.ilimit)
  o.d = (o.dconst*o.d - (process_value - o.process_value))
      / (o.dconst + dt + 1e-12)
  o.process_value = process_value
--return control output
  return (o.pgain*o.p + o.igain*o.i + o.dgain*o.d)
end

mt.__index = {
  set_gains = set_gains,
  set_pgain = set_pgain,
  set_igain = set_igain,
  set_dgain = set_dgain,
  set_ilimit = set_ilimit,
  set_dconst = set_dconst,
  reset = reset,
  update = update,
}
