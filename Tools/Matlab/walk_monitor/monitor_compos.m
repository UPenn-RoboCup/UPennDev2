%addpath(genpath('.'));
addpath(genpath('../'));
clf
clear all

a=shm('mcmStatus01sj');


f_handle_1 = figure()

t_old = 0;
t_nonupdate_count = 0;
should_clear = false;

while 1 

  com_poses = a.get_com_pos();
  com_x = com_poses(1:22);
  com_y = com_poses(23:44);
  com_z = com_poses(45:66);
  
  
  plot3(com_x([22 1 2 3]),com_y([22 1 2 3]),com_z([22 1:3]));
  hold on;
  
  plot3(com_x([22,7:9]),com_y([22 7 8 9]),com_z([22 7:9]));

  
  plot3(com_x([22 21 13:16]),com_y([22 21 13:16]),com_z([22 21 13:16]));
  
  plot3(com_x([21,17:20]),com_y([21 17:20]),com_z([21 17:20]));
  axis([-1 1 -1 1 0 2]);
  hold off;
  drawnow;
  pause(0.01);
end