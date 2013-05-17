local Spacenav = require('spacenav')
local ret = Spacenav.open()
print('ret',ret)

cntr = 0;
while (true) do
  cntr = cntr + 1;

  local tbl = Spacenav.get()
	if tbl then
		print('Got something!')
	end
  if type(tbl) == 'table' then
    if tbl.event == 'motion' then
      print(tbl.x, tbl.y, tbl.z, tbl.rx, tbl.ry, tbl.rz)
    elseif tbl.event == 'button' then
      print(tbl.bnum, tbl.bpress)
    end
  end
end
