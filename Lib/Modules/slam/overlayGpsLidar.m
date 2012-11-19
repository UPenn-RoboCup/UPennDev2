load GPS
load TRAJ

plot(GPS.traj(1,:),GPS.traj(2,:),'b.'); hold on;

traj= [TRAJ.traj(1,:); TRAJ.traj(2,:); zeros(size(TRAJ.traj(1,:))); ones(size(TRAJ.traj(1,:)))];

T=trans([483767.187537-40 4422508.916057-35 0])*rotz(1.2);

traj2=T*traj;

plot(traj2(1,:),traj2(2,:),'r.'); hold off;