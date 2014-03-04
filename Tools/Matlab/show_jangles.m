function show_jangles(traj)
  t=traj(:,1);
  %1 - t
  % 2 3 4 5 6 7 8: jAngle
  
 
  subplot(3,1,1);
  plot(traj(:,9:11));  %XYZ
 
  subplot(3,1,2);
  plot(traj(:,12:14)*180/pi);  %RPY
  
  
  subplot(3,1,3);
   plot(traj(:,2:8)); %jangles
  drawnow;
end