require('numlua')

----------------------------------------------------------------------
-- linreg : linear regression and prediction
----------------------------------------------------------------------

linreg = {}
linreg.__index = linreg

local function gaussian(x, center, width)
  return math.exp(-width*(x - center)^2)
end

function linreg.new(psi)
  -- new linear regression
  local o = {}
  o.psi = psi or {} -- table of basis functions
  o.w = {}          -- table of basis weights
  for i = 1,#psi do
    o.w[i] = 0
  end
  return setmetatable(o, linreg)
end

function linreg.new_rbf(centers, widths)
  -- new linear regression using radial basis functions
  local psi = {}
  for i = 1,#centers do
    psi[i] = function (x)
      return gaussian(x, centers[i], widths[i])
    end
  end
  return linreg.new(psi)
end

function linreg.get_basis_functions(o)
  return o.psi
end

function linreg.get_weights(o)
  return o.w
end

function linreg.set_basis_functions(o, psi)
  o.psi = psi
end

function linreg.set_weights(o, w)
  o.w = w
end

function linreg.fit(o, ydata, xdata)
  -- calculate basis weights via ordinary least squares
  local y = ydata
  if (type(y) == 'table') then
    y = matrix.fromtable(y)
  end

  local PSI = matrix.new(#xdata, #o.psi)
  for i = 1,#xdata do
    for j = 1,#o.psi do
      PSI[i][j] = o.psi[j](xdata[i])
    end
  end

  local w = PSI % y
  o.w = w:totable()
end

function linreg.predict(o, x)
  -- predict y = f(x) using linear regression
  local w = matrix.fromtable(o.w)
  local b = matrix.new(#o.psi)
  for i = 1,#o.psi do
    b[i] = o.psi[i](x)
  end
  return matrix.dot(w, b)
end

return linreg
