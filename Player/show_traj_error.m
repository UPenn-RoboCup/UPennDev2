function show_traj_error(traj)
  close all;
  subplot(3,1,1);
  t=traj(:,1);
  %1 - t
  % 2 3 4 5 6: LLeg
  % 7 8 9 10 11: LLegCommand
  % 12 13: roll and pitch
  
  plot(t,traj(:,2)-traj(:,7),'g', t,traj(:,6)-traj(:,11),'b', ...
      t,traj(:,2)-traj(:,7)+traj(:,6)-traj(:,11),'r--',  ...
      t,traj(:,12),'r');
  legend('Hip Roll Error','Ankle Roll Error', 'Total Roll Error', 'IMU Roll Error');
  
  
  %LLEG ERROR
  %{
  plot(t,traj(:,14)-traj(:,16),'g',t,traj(:,15)-traj(:,17),'b');
  legend('LHipRoll','LHipPitch');
  %}
  subplot(3,1,2);
 
  
  
  plot(t,traj(:,3)-traj(:,8),'g', t,traj(:,4)-traj(:,9),'b', t,traj(:,5)-traj(:,10),'k',...      
      t,traj(:,3)+traj(:,4)+traj(:,5)-traj(:,8)-traj(:,9)-traj(:,10),'r--',  ...
      t,traj(:,13)-10,'r');
  legend('Hip Pitch Error','Knee Pitch Error', 'Ankle Pitch Error', 'Total Pitch Error', 'IMU Pitch Error');
  
  t_siz = size(t,1)
  t_delay = t - [0; t(1:t_siz-1)];
  
  size(t_delay)
  
  subplot(3,1,3);
  plot(t,t_delay*1000)
  legend('tStep')
  
  drawnow;
end