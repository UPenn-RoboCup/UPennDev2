dofile'../../include.lua'
local Body = require'Body'
while true do
  local pos_tbl = {}
  --
  local head = Body.get_head_position()*Body.RAD_TO_DEG
  table.insert(pos_tbl,'Head: '..tostring(head))
  --
  local larm = Body.get_larm_position()*Body.RAD_TO_DEG
  table.insert(pos_tbl,'Left arm: '..tostring(larm))
  --
  local lgrip = Body.get_lgrip_position()*Body.RAD_TO_DEG
  table.insert(pos_tbl,'Left grip: '..tostring(lgrip))
  --
  local lidar = Body.get_lidar_position()*Body.RAD_TO_DEG
  table.insert(pos_tbl,'LIDAR: '..tostring(lidar))

  -- Make requests
  Body.request_head_position()
  Body.request_larm_position()
  Body.request_lgrip_position()
  Body.request_lidar_position()
  -- Print
  os.execute('clear')
  print('Positions')
  print(table.concat(pos_tbl,'\n'))
  -- Sleep
  unix.usleep(1e5)
end