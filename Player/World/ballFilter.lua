local ballFilter = {}

local vector = require'vector'
local util = require'util'
require'wcm'
mod_angle = util.mod_angle


function ballFilter.reset()
  wcm.set_ballfilter_r(1)
  wcm.set_ballfilter_a(0)
  wcm.set_ballfilter_rvar(1E10)
  wcm.set_ballfilter_avar(1E10)
end

function ballFilter.observation_ra(r1,a1,rErr,aErr,t)
  local r = wcm.get_ballfilter_r()
  local a = wcm.get_ballfilter_a()
  local rvar = wcm.get_ballfilter_rvar()
  local avar = wcm.get_ballfilter_avar()

  rErr = rErr or 1
  aErr = aErr or 1
  local rvar1 = rErr^2
  local avar1 = aErr^2

  local dr = r1 - r
  r = r + (rvar * dr)/(rvar + rvar1)
  local da = mod_angle(a1 - a)
  a = a + (avar * da)/(avar + avar1)

  rvar = (rvar * rvar1)/(rvar + rvar1)
  if (rvar + rvar1 < dr^2) then rvar = dr^2 end

  avar = (avar * avar1)/(avar + avar1)
  if (avar + avar1 < da^2) then
    avar = da^2
  end

  local x = r * math.cos(a)
  local y = r * math.sin(a)

  wcm.set_ballfilter_r(r)
  wcm.set_ballfilter_a(a)
  wcm.set_ballfilter_rvar(rvar)
  wcm.set_ballfilter_avar(avar)
  wcm.set_ball_x(x)
  wcm.set_ball_y(y)
  wcm.set_ball_t(t)
  --print("ball pos:",x,y)
end

function ballFilter.observation_xy(x,y,rErr,aErr,t)
  rErr = rErr or 1
  aErr = aErr or 1
  local r = math.sqrt(x^2 + y^2)
  local a = math.atan2(y, x)
  return ballFilter.observation_ra(r, a, rErr, aErr,t)
end

-- TODO: not in need
function ballFilter.odometry(dx, dy, da, drErr, daErr)
  local r = wcm.get_ballfilter_r()
  local a = wcm.get_ballfilter_a()
  local rvar = wcm.get_ballfilter_rvar()
  local avar = wcm.get_ballfilter_avar()

  local x = r * math.cos(a) - dx
  local y = r * math.sin(a) - dy
  r = math.sqrt(x^2 + y^2)
  a = mod_angle(math.atan2(y,x) - da)

  drErr = drErr or 0.10 * math.sqrt(dx^2 + dy^2)
  daErr = daErr or 0.10 * math.abs(da)

  wcm.set_ballfilter_r(r)
  wcm.set_ballfilter_a(a)
  wcm.set_ballfilter_rvar(rvar+drErr)
  wcm.set_ballfilter_avar(avar+daErr)
  wcm.set_ball_x(x)
  wcm.set_ball_y(y)
end


ballFilter.reset()
return ballFilter
