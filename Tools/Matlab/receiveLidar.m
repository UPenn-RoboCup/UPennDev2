%robot = shm_robot(1,1);

while(1)
      range = typecast(robot.lidar.get_ranges() ,'SINGLE');
      t = robot.lidar.get_timestamp();
      c = robot.lidar.get_counter();
      fprintf(1,'Time: %.4f, Count: %d\n',t,c);
      
      plot(range, '.');
      drawnow;
      %fprintf(1,'.');
      pause(1/40);
end
