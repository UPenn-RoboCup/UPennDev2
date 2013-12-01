local spacemouse = require 'spacemouse'
spacemouse.lsusb()
--sm = spacemouse.init(0x046d, 0xc62b) --pro
sm = spacemouse.init(0x046d, 0xc626) --regular

local cnt = 0
while true do

  local event, data = sm:get()
  if event=='button' and true then
    io.write('\nButton ',data)
    io.flush()
  elseif event=='translate' and false then
    io.write('\nTranslate\n')
    for k,v in pairs(data) do io.write('\t',k,': ',v) end
    io.flush()
  elseif event=='rotate' and false then
    io.write('\nRotate\n')
    for k,v in pairs(data) do io.write('\t',k,': ',v) end
    io.flush()
  end

  --[[
  local raw = sm:get_raw()
  if raw then
    --print( raw:byte(1,#raw) )
    print( raw )
    print()
  end
  --]]
end
