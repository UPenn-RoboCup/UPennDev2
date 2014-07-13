local obstacleFilter = {}

local util = require'util'
local mod_angle = util.mod_angle
require'wcm'



local function observation_ra(self, r1, a1, rErr, aErr)
  local rErr = rErr or 1
  local aErr = aErr or 1
  local rvar1 = rErr^2
  local avar1 = aErr^2

  local dr = r1 - self.r
  self.r = self.r + (self.rvar * dr)/(self.rvar + rvar1)
  local da = mod_angle(a1 - self.a)
  self.a = self.a + (self.avar * da)/(self.avar + avar1)

  self.rvar = (self.rvar * rvar1)/(self.rvar + rvar1)
  if (self.rvar + rvar1 < dr^2) then self.rvar = dr^2 end

  self.avar = (self.avar * avar1)/(self.avar + avar1)
  if (self.avar + avar1 < da^2) then self.avar = da^2 end

  local x = self.r * math.cos(self.a)
  local y = self.r * math.sin(self.a)
  local t = t

	return x, y
  --wcm['set_obstacle_v'..self.id]({x, y})
  --print(string.format("OBS %d: %.2f   %.2f", self.id, x, y))
end

local function observation_xy(self, x, y, rErr, aErr)
  local r = math.sqrt(x^2 + y^2)
  local a = math.atan2(y, x)
  return observation_ra(self, r, a, rErr, aErr)
end

-- TODO: not in need?
local function odometry(self, dx, dy, da, drErr, daErr)
  local x = self.r * math.cos(self.a) - dx
  local y = self.r * math.sin(self.a) - dy
  self.r = math.sqrt(x^2 + y^2)
  self.a = mod_angle(math.atan2(y,x) - da)

  local drErr = drErr or 0.10 * math.sqrt(dx^2 + dy^2)
  local daErr = daErr or 0.10 * math.abs(da)
  self.rvar = self.rvar + drErr
  self.avar = self.avar + daErr

  --wcm['set_obstacle_v'..self.id]({x, y})
end

function obstacleFilter.new(id)
  -- Set up the object
  local f = {}
  -- Variables
  f.r = 1
  f.a = 0
  f.rvar = 1e10
  f.avar = 1e10
  f.id = id
  -- Methods
  f.observation_ra = observation_ra
  f.observation_xy = observation_xy
  f.odometry = odometry
  return f
end

return obstacleFilter
