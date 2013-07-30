local spacemouse = require 'spacemouse'
spacemouse.lsusb()

--sm = spacemouse.init(0x046d, 0xc62b) --pro
sm = spacemouse.init(0x046d, 0xc626) --regular

local cnt = 0
while true do
  tbl = sm:get()
  if tbl and tbl.event then
    local evt = false
    for k,v in pairs(tbl) do
      if v~=0 and k~='event' then evt = true end
      if evt then
        io.write("\n\n",cnt," | ",tbl.event," (",k,"): ",v)
      end
    end
    if evt then
      io.flush()
      cnt = cnt+1
    end
  end
end
