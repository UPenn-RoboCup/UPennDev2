dofile('../include.lua')

require('trajectory')
require('gnuplot')

x0 = {0, 1, 0}
x1 = {1, 10, 0}
tau = 2


local mjt = trajectory.minimum_jerk(x0, x1, tau)

x = {0}
xd = {0}
xdd = {0}

for i = 1,100 do
  x[i], xd[i], xdd[i] = mjt(tau*i/100)
end

gnuplot.figure()
gnuplot.plot(x)
gnuplot.figure()
gnuplot.plot(xd)
gnuplot.figure()
gnuplot.plot(xdd)
