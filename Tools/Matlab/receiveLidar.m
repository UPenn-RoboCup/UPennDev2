robot = shm_robot(0,1);

while(1)
      plot(robot.hokuyo.get_ranges(), '.');
      drawnow;
      %fprintf(1,'.');
end
