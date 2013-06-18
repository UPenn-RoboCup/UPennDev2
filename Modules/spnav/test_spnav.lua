local spnav = require 'spnav'

--spnav.lsusb()

spMousePro_ud = spnav.init(0x046d, 0xc62b)
spNavigator_ud = spnav.init(0x046d, 0xc626)

while true do
--  tbl = spnav_ud:get()
--  if tbl then
--    if tbl.event == 'motion' then
--      print(tbl.x, tbl.y, tbl.z, tbl.rx, tbl.ry, tbl.rz)
--    elseif tbl.event == 'button' then
--      print(tbl.bnum, tbl.bpress)
--    end
--  end

  str = spMousePro_ud:get_raw()
  if #str > 0 then
    print("SpaceMouse Pro", #str, str)
  end

  str = spNavigator_ud:get_raw()
  if #str > 0 then
    print("SpaceNavigator", #str, str)
  end

end
