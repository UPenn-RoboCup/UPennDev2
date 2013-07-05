module(..., package.seeall);

require('lcm')
require('unix')
Config = require('ConfigPenn')
Body = require(Config.Body);


--Head lidar
lidar0={};
lidar0.move_dir = 1;
lidar0.pan_active = Config.movement.lidar0.pan_active;
lidar0.angle_step = Config.movement.lidar0.angle_step;
lidar0.angle0 = Config.movement.lidar0.angle0;
lidar0.angle1 = Config.movement.lidar0.angle1;

lidar0.angle_mag = lidar0.angle1-lidar0.angle0;
lidar0.angle_vel = lidar0.angle_step * 40;
lidar0.scan_count = 0;
lidar0.t0 = 0;

neck ={0,0};


function entry()
  print(_NAME..' Entry' ) 

  t = unix.time();
  lcm:set_head_lidar_panning(0);
  lidar0.move_dir = 1;
  lidar0.angle = lidar0.angle0;
  lidar0.t0 = t;
  lidar0.scan_count = lcm:get_head_lidar_scan_count()[1]
  Body.set_neck_target_position({0,lidar0.angle});
end

function update()
--  print(_NAME..' Update' ) 

  lcm:set_head_lidar_panning(1);
  local done = update_lidar(lidar0,0);
  if done then
    return "done";
  else

  end
  Body.set_neck_position({0,lidar0.angle});
  
end

function update_lidar(lidar_st, lidarindex)
  local t = unix.time()
  local scan_duration = lidar_st.angle_mag / lidar_st.angle_vel ;
  if t- lidar_st.t0 > scan_duration then
    lidar_st.scan_count = lidar_st.scan_count+1;
    lidar_st.t0 = t;
    if lidarindex==0 then
      lcm:set_head_lidar_scan_count(lidar_st.scan_count);
    else
      lcm:set_chest_lidar_scan_count(lidar_st.scan_count);
    end
    return true;
  end
  local ph0 = (t-lidar_st.t0) / scan_duration;
  if lidar_st.move_dir==1 then
    lidar_st.angle = lidar_st.angle0 + lidar_st.angle_mag  * ph0;
  else
    lidar_st.angle = lidar_st.angle0 + lidar_st.angle_mag  * (1 - ph0);
  end
  return false;
end


function exit()
  lcm:set_head_lidar_panning(0);
  print(_NAME..' Exit' ) 
end
