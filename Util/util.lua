local util = {}
local vector = require'vector'

local abs = math.abs

function util.mod_angle(a)
  -- Reduce angle to [-pi, pi)
  local b = a % (2*math.pi)
  if b >= math.pi then return (b - 2*math.pi) end
  return b
end


function util.sign(x)
  -- return sign of the number (-1, 0, 1)
  if x > 0 then return 1
  elseif x < 0 then return -1
  else return 0
  end
end

function util.min(t)
  -- find the minimum element in the array table
  -- returns the min value and its index
  local imin = 0
  local tmin = math.huge
  for i=1,#t do
    local v = t[i]
    if v < tmin then
      tmin = v
      imin = i
    end
  end
  return tmin, imin
end

function util.max(t)
  -- find the maximum element in the array table
  -- returns the min value and its index
  local imax = 1 --0
  local tmax = -1*math.huge
  for i=1,#t do
    if t[i] > tmax then
      tmax = t[i]
      imax = i
    end
  end
  return tmax, imax
end

function util.se2_interpolate(t, u1, u2)
  -- helps smooth out the motions using a weighted average
  return vector.new{
    u1[1]+t*(u2[1]-u1[1]),
    u1[2]+t*(u2[2]-u1[2]),
    u1[3]+t*util.mod_angle(u2[3]-u1[3])
  }
end

function util.se3_interpolate(t, u1, u2, u3)
  --Interpolation between 3 xya values
  if t<0.5 then
    local tt=t*2
    return vector.new{
      u1[1]+tt*(u2[1]-u1[1]),
      u1[2]+tt*(u2[2]-u1[2]),
      u1[3]+tt*util.mod_angle(u2[3]-u1[3])
    }
  else
    local tt=t*2-1
    return vector.new{
      u2[1]+tt*(u3[1]-u2[1]),
      u2[2]+tt*(u3[2]-u2[2]),
      u2[3]+tt*util.mod_angle(u3[3]-u2[3])
    }
  end
end

function util.shallow_copy(a)
  --copy the table by value
  local ret={}
  for k,v in pairs(a) do ret[k]=v end
  return ret
end


--Piecewise linear function for IMU feedback
local function procFunc(a,deadband,maxvalue)
  local b = math.min( math.max(0,math.abs(a)-deadband), maxvalue)
  if a<=0 then return -b end
  return b
end

function util.clamp_vector(values,min_values,max_values)
	local clamped = vector.new()
	for i,v in ipairs(values) do
		clamped[i] = math.max(math.min(v,max_values[i]),min_values[i])
	end
	return clamped
end

-- Tolerance approach to a vector
-- Kinda like a gradient descent
function util.approachTol( values, targets, speedlimits, dt, tolerance )
  tolerance = tolerance or 1e-6
  -- Tolerance check (Asumme within tolerance)
  local within_tolerance = true
  -- Iterate through the limits of movements to approach
  
  --SJ: Now can be used for scalar
  if type(values)=="table" then
    for i,speedlimit in ipairs(speedlimits) do
      -- Target value minus present value
      local delta = targets[i] - values[i]
      -- If any values is out of tolerance,
      -- then we are not within tolerance
      if math.abs(delta) > tolerance then
        within_tolerance = false
        -- Ensure that we do not move motors too quickly
        delta = util.procFunc(delta,0,speedlimit*dt)
        values[i] = values[i]+delta
      end
    end
  else
    local delta = targets - values      
      if math.abs(delta) > tolerance then
        within_tolerance = false
        -- Ensure that we do not move motors too quickly
        delta = util.procFunc(delta,0,speedlimits*dt)
        values = values+delta
      end
  end
  -- Return the next values to take and if we are within tolerance
  return values, within_tolerance
end

-- Tolerance approach to a vector
-- Kinda like a gradient descent

--SJ: This approaches to the DIRECTION of the target position

function util.approachTolTransform( values, targets, vellimit, dt, tolerance )
  tolerance = tolerance or 1e-6
  -- Tolerance check (Asumme within tolerance)
  local within_tolerance = true
  -- Iterate through the limits of movements to approach
  
  local linearvellimit = vellimit[1] --hack for now

  local cur_pos = vector.slice(values,1,3)  
  local target_pos = vector.slice(targets,1,3)  
  local delta = target_pos - cur_pos
  local mag_delta = math.sqrt(delta[1]*delta[1] + delta[2]*delta[2] + delta[3]*delta[3])
  
  if math.abs(mag_delta)>tolerance then
    movement = math.min(mag_delta, linearvellimit*dt)
    values[1] = values[1] + delta[1]/mag_delta * movement 
    values[2] = values[2] + delta[2]/mag_delta * movement 
    values[3] = values[3] + delta[3]/mag_delta * movement 
    within_tolerance = false
  end
  

  for i=4,6 do --Transform 
    -- Target value minus present value
    local delta = targets[i] - values[i]    
    if math.abs(delta) > tolerance then
      within_tolerance = false
      -- Ensure that we do not move motors too quickly
      delta = util.procFunc(delta,0,vellimit[i]*dt)
      values[i] = values[i]+delta
    end    
  end
  
  -- Return the next values to take and if we are within tolerance
  return values, within_tolerance
end

function util.approachTolWristTransform( values, targets, vellimit, dt, tolerance )
  tolerance = tolerance or 1e-6
  -- Tolerance check (Asumme within tolerance)
  local within_tolerance = true
  -- Iterate through the limits of movements to approach
  
  for i=4,6 do --Transform 
    -- Target value minus present value
    local delta = targets[i] - values[i]    
    if math.abs(delta) > tolerance then
      within_tolerance = false
      -- Ensure that we do not move motors too quickly
      delta = util.procFunc(delta,0,vellimit[i]*dt)
      values[i] = values[i]+delta
    end    
  end
  
  -- Return the next values to take and if we are within tolerance
  return values, within_tolerance
end


-- Tolerance approach to a radian value (to correct direction)
-- Kinda like a gradient descent
--SJ: modangle didnt work for whatever reason so just used math.mod


function util.approachTolRad( values, targets, speedlimits, dt, tolerance )
  tolerance = tolerance or 1e-6
  -- Tolerance check (Asumme within tolerance)
  local within_tolerance = true
  -- Iterate through the limits of movements to approach
  for i,speedlimit in ipairs(speedlimits) do
    -- Target value minus present value
    local delta = util.mod_angle(targets[i]-values[i])
--    local delta = math.mod(targets[i]-values[i]+5*math.pi,2*math.pi)-math.pi
    -- If any values is out of tolerance,
    -- then we are not within tolerance
    if math.abs(delta) > tolerance then
      within_tolerance = false
      -- Ensure that we do not move motors too quickly
      delta = util.procFunc(delta,0,speedlimit*dt)
--      values[i] = math.mod(values[i]+delta+5*math.pi,2*math.pi)-math.pi
      values[i] = values[i] + delta
    end
  end
  -- Return the next values to take and if we are within tolerance
  return values, within_tolerance
end

-- Tolerance approach to a vector
-- Kinda like a gradient descent
function util.goto(cur, target, step, tolerance)
  -- Tolerance check (Asumme within tolerance)
  local within_tolerance = true
  -- Iterate through the limits of movements to approach
  local next
  --SJ: Now can be used for scalar
  if type(cur)=="table" then
    next = vector.copy(cur)
    for i, s in ipairs(step) do
      -- Target value minus present value
      local delta = target[i] - cur[i]
      -- If any values is out of tolerance,
      -- then we are not within tolerance
      if abs(delta) > tolerance[i] then
        within_tolerance = false
        -- Ensure that we do not move motors too quickly
        delta = procFunc(delta, 0, s)
        next[i] = cur[i] + delta
      end
    end
  else
    local delta = target - values
    next = cur
    if abs(delta) > tolerance then
      within_tolerance = false
      -- Ensure that we do not move motors too quickly
      delta = procFunc(delta, 0, step)
      next = cur + delta
    end
  end
  -- Return the next values to take and if we are within tolerance
  return within_tolerance and target or next, within_tolerance
end


function util.pose_global(pRelative, pose)
  local ca = math.cos(pose[3])
  local sa = math.sin(pose[3])
  return vector.pose{pose[1] + ca*pRelative[1] - sa*pRelative[2],
                    pose[2] + sa*pRelative[1] + ca*pRelative[2],
--                    util.mod_angle(pose[3] + pRelative[3])}
                    pose[3] + pRelative[3]}

--SJ: Using modangle here makes the yaw angle jump which kills walk 

end

function util.pose_relative(pGlobal, pose)
  local ca = math.cos(pose[3])
  local sa = math.sin(pose[3])
  local px = pGlobal[1]-pose[1]
  local py = pGlobal[2]-pose[2]
  local pa = pGlobal[3]-pose[3]
  return vector.pose{ca*px + sa*py, -sa*px + ca*py, util.mod_angle(pa)}
end

---table of uniform distributed random numbers
--@param n length of table to return
--@return table of n uniformly distributed random numbers
function util.randu(n)
  local t = {}
  for i = 1,n do t[i] = math.random() end
  return t
end

---Table of normal distributed random numbers.
--@param n length of table to return
--@return table of n normally distributed random numbers
function util.randn(n)
  local t = {}
  for i = 1,n do
    --Inefficient implementation:
    t[i] = math.sqrt(-2.0*math.log(1.0-math.random())) *
                      math.cos(math.pi*math.random())
  end
  return t
end

function util.factorial(n)
  if n == 0 then
    return 1
  else
    return n * factorial(n - 1)
  end
end

function util.polyval_bz(alpha, s)
  local b = 0
  local M = #alpha-1
  for k =0,M do
    b = b + alpha[k+1] * factorial(M)/(factorial(k)*factorial(M-k)) * s^k * (1-s)^(M-k)
  end
  return b
end

function util.bezier( alpha, s )

  local n = #alpha
  local m = #alpha[1]
  local M = m-1

  -- Pascal's triangle
  local k = {}
  if M==3 then
    k={1,3,3,1}
  elseif M==4 then
    k={1,4,6,4,1}
  elseif M==5 then
    k={1,5,10,10,5,1}
  elseif M==6 then
    k={1,6,15,20,15,6,1}
  end
  
  local x = vector.ones(M+1)
  local y = vector.ones(M+1)
  for i=1,M do
    x[i+1] = s*x[i]
    y[i+1] = (1-s)*y[i]
  end
  
  local value = vector.zeros(n)
  for i=1,n do
    --value[i] = 0
    for j=1,M+1 do value[i] = value[i] + alpha[i][j]*k[j]*x[j]*y[M+2-j] end
  end

  return value
end

function util.tablesize(table)
  local count = 0
  for _ in pairs(table) do count = count + 1 end
  return count 
end

function util.ptable(t)
  -- print a table key, value pairs
  for k,v in pairs(t) do print(k,v) end
end

function util.ptorch(data, W, Precision)
  local w = W or 5
  local precision = Precision or 10
  local torch = require'torch'
  local tp = type(data)
  if tp == 'userdata' then
    tp = torch.typename(data) or ''
    local dim = data:dim()
    local row = data:size(1)
    local col = 1
    if dim == 1 then
      for i = 1, row do print(data[i]) end
      print('\n'..tp..' - size: '..row..'\n')
    elseif dim == 2 then 
      col = data:size(2) 
      for r = 1, row do
        for c = 1, col do
          io.write(string.format("%"..w.."."..precision.."f",data[r][c])..' ')
        end
        io.write('\n')
      end
      print('\n'..tp..' - size: '..row..'x'..col..'\n')
    else
      print'Printing torch objects with more than 2 dimensions is not support'
    end
  else
    print(data)
  end
  io.flush()
end

--https://en.wikipedia.org/wiki/ANSI_escape_code#Colors
--[[
Color table[7]
Intensity 0 1 2 3 4 5 6 7
Normal  Black Red Green Yellow  Blue  Magenta Cyan  White
Bright  Black Red Green Yellow  Blue  Magenta Cyan  White
--]]
local ctable = {
  ['black'] = 0,
  ['red'] = 1,
  ['green'] = 2,
  ['yellow'] = 3,
  ['blue'] = 4,
  ['magenta'] = 5,
  ['cyan'] = 6,
  ['white'] = 7,
}
local color_end = '\027[0m'
--if blink then "\027[31;5m" end

util.color = function(str,fg,bg,blink)
  assert(ctable[fg],string.format('Foreground Color %s does not exist',fg))
  local begin_fg = string.format('\027[%dm',30+ctable[fg])
  if bg then
    assert(ctable[bg],string.format('Background Color %s does not exist',bg))
    local begin_bg = string.format('\027[%dm',40+ctable[bg])
    return begin_bg..begin_fg..str..color_end
  end
  return begin_fg..str..color_end
end

util.procFunc = procFunc

return util
