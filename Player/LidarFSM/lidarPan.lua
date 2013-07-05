module(..., package.seeall);

require('lcm')
require('unix')
Config = require('ConfigPenn')
Body = require(Config.Body);


--Chest lidar
lidar1={};
lidar1.move_dir = 1;
lidar1.pan_active = Config.movement.lidar1.pan_active;
lidar1.angle_step = Config.movement.lidar1.angle_step;
lidar1.angle0 = Config.movement.lidar1.angle0;
lidar1.angle1 = Config.movement.lidar1.angle1;

lidar1.angle_mag = lidar1.angle1-lidar1.angle0;
lidar1.angle_vel = lidar1.angle_step * 40;
lidar1.scan_count = 0;
lidar1.t0 = 0;

function entry()
  print(_NAME..' Entry' ) 

  t = unix.time();
  lcm:set_chest_lidar_panning(0);
  lidar1.angle = lidar1.angle0;
  lidar1.t0 = t;
  lidar1.scan_count = lcm:get_chest_lidar_scan_count()[1]
  Body.set_lidar_position({lidar1.angle});
end

function update()
--  print(_NAME..' Update' ) 
  lcm:set_chest_lidar_panning(1);
  update_lidar(lidar1,1);
  Body.set_lidar_position({lidar1.angle});  
end

function update_lidar(lidar_st, lidarindex)
  local t = unix.time()
  local scan_duration = lidar_st.angle_mag / lidar_st.angle_vel ;
  if t- lidar_st.t0 > scan_duration then
    lidar_st.move_dir = -lidar_st.move_dir;
    lidar_st.scan_count = lidar_st.scan_count+1;
    lidar_st.t0 = t;
    if lidarindex==0 then
      lcm:set_head_lidar_scan_count(lidar_st.scan_count);
    else
      lcm:set_chest_lidar_scan_count(lidar_st.scan_count);
    end
  end
  local ph0 = (t-lidar_st.t0) / scan_duration;
  if lidar_st.move_dir==1 then
    lidar_st.angle = lidar_st.angle0 + lidar_st.angle_mag  * ph0;
  else
    lidar_st.angle = lidar_st.angle0 + lidar_st.angle_mag  * (1 - ph0);
  end
end


function exit()
  lcm:set_chest_lidar_panning(0);
  print(_NAME..' Exit' ) 
end
