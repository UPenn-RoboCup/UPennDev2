package.path = '../../Framework/Util/?.lua;'..package.path

require('gnuplot')

if (not arg[1]) then
  print('usage : lua plot_logs.lua [file1] [file2] ...')
end

for i = 1, #arg do
  local data = {}
  for line in io.lines(arg[i]) do
    data[#data + 1] = tonumber(line)
  end
  gnuplot.figure()
  gnuplot.plot(data, '-')
  gnuplot.title(arg[i])
  gnuplot.grid(true)
end
