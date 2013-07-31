--https://en.wikipedia.org/wiki/ANSI_escape_code#Colors
--[[
Color table[7]
Intensity	0	1	2	3	4	5	6	7
Normal	Black	Red	Green	Yellow	Blue	Magenta	Cyan	White
Bright	Black	Red	Green	Yellow	Blue	Magenta	Cyan	White
--]]
local colors = {}

colors.print_red = function(str)
  io.write("\027[31;5m")-- blick
  io.write('')
  io.write('\n')
  io.flush()
end

colors.print_blink_red = function(str)
  io.write("\027[31;5m")-- ;5 means blink
  io.write(str)
  io.write('\027[0m')
  io.write('\n')
  io.flush()
end

colors.print_white_on_red = function(str)
  io.write("\027[41m")-- ;5 means blink
  io.write("\027[37m")-- ;5 means blink
  io.write(str)
  io.write('\027[0m')
  io.write('\n')
  io.flush()
end

return colors