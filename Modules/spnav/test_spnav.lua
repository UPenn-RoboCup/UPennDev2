local spacemouse = require 'spacemouse'

--spnav.lsusb()

--sm = spacemouse.init(0x046d, 0xc62b)
sm = spacemouse.init(0x046d, 0xc626)

local cnt = 0
while true do
  tbl = sm:get()
  if tbl and tbl.event then
    local evt = false
    for k,v in pairs(tbl) do
      if v~=0 and k~='event' then
        io.write("\n\n",cnt," | ",tbl.event," (",k,"): ",v)
        evt = true
      end
    end
    if evt then cnt = cnt+1 end
  end
  

  --[[
  str = sm:get_raw()
  if #str > 0 then
    print("SpaceMouse Pro", #str, str)
  end
  --]]

  --[[
  str = sm:get_raw()
  if #str > 0 then
    print("SpaceNavigator", #str, str)
  end
  --]]
  io.flush()
end
