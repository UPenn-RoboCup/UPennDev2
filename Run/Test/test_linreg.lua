dofile('../include.lua')

require('unix')
require('numlua')
require('linreg')
require('gnuplot')

local nbasis  = 10
local t = matrix.seq(0, 1000)/1000

-- distribute radial basis functions logarithmically on (0, 1)
local centers, widths = {}, {}
for i = 1,nbasis do
  centers[i] = 1 - math.log10(9*(nbasis - i)/nbasis + 1)
  widths[i]  = 0.5*(centers[i] - (centers[i-1] or 0))^(-2)
end

-- create linear regression object
local r = linreg.new_rbf(centers, widths)

-- define target function
function target(s)
  return math.sin(2*math.pi*s)
end
local x_target = t:copy()
x_target:map(target)

-- fit data
r:fit(x_target, t)

-- predict target function
function prediction(s)
  return r:predict(s)
end
local x_learned = t:copy()
x_learned:map(prediction)

gnuplot.figure()
gnuplot.plot(
  {'target function', t, x_target, '~'}, 
  {'learned function', t, x_learned, '~'}
)
