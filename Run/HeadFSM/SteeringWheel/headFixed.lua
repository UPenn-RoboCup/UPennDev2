module(..., package.seeall);

function entry()
  lcm:set_head_lidar_panning(0);
  print(_NAME..' Entry' ) 

end

function update()
--  print(_NAME..' Update' ) 

end

function exit()
  lcm:set_head_lidar_panning(0);
  print(_NAME..' Exit' ) 
end
