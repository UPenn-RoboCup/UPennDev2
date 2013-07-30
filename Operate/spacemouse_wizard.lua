dofile'include.lua'
local spacemouse = require 'spacemouse'
--sm = spacemouse.init(0x046d, 0xc62b) -- pro
local sm = spacemouse.init(0x046d, 0xc626) -- regular

local cnt = 0
while true do
  local tbl = sm:get()
  local is_event = false
  
  if tbl then    
    for k,v in pairs(tbl) do
      if v~=0 and k~='event' then is_event = true end
      if is_event then
        io.write("\n\n",cnt," | ",tbl.event," (",k,"): ",v)
      end
    end
    if is_event then
      io.flush()
      cnt = cnt+1
    end
  end
end