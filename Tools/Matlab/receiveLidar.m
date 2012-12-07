robot = shm_robot(18,1);

while(1)
      range = typecast(robot.lidar.get_ranges() ,'SINGLE');
      plot(range, '.');
      drawnow;
      %fprintf(1,'.');
end
