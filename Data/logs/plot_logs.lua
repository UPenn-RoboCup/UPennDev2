package.path = '../../Framework/Util/?.lua;'..package.path

require('gnuplot')

if (arg[1] == '-h') then
  hold = true
  table.remove(arg, 1)
end

if (not arg[1]) then
  print('usage : lua plot_logs.lua [file1] [file2] ...')
end

local plot_args = {}
for i = 1, #arg do
  local data = {}
  for line in io.lines(arg[i]) do
    data[#data + 1] = tonumber(line)
  end
  plot_args[i] = {string.match(arg[i], '([^.]*)'), data, '-'}
end

if hold then
  gnuplot.figure()
  gnuplot.plot(unpack(plot_args))
  gnuplot.grid(true)
else 
  for i = 1, #arg do
    gnuplot.figure()
    gnuplot.plot(plot_args[i])
    gnuplot.grid(true)
  end
end
