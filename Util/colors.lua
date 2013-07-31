--https://en.wikipedia.org/wiki/ANSI_escape_code#Colors
--[[
Color table[7]
Intensity	0	1	2	3	4	5	6	7
Normal	Black	Red	Green	Yellow	Blue	Magenta	Cyan	White
Bright	Black	Red	Green	Yellow	Blue	Magenta	Cyan	White
--]]
local colors = {}
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

colors.wrap = function(str,fg,bg,blink)
  assert(ctable[fg],string.format('Foreground Color %s does not exist',fg))
  local begin_fg = string.format('\027[%dm',30+ctable[fg])
  if bg then
    assert(ctable[bg],string.format('Background Color %s does not exist',bg))
    local begin_bg = string.format('\027[%dm',40+ctable[bg])
    return begin_bg..begin_fg..str..color_end
  end
  return begin_fg..str..color_end
end

return colors