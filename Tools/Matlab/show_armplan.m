%% Plot the joint trajectories in time
qq = reshape(q, [nq, np])';
qq0 = reshape(qPath0, [nq, np])';
nt = dt * np;
t = 0:dt:nt-dt;
tlim = [t(1), t(end)];

figure(1);
plot(t, rad2deg(qq0));
xlim(tlim);
xlabel('Time (s)');
ylim([-180, 180]);
ylabel('Degrees');
title('Original Trajectory');

figure(2);
plot(t, rad2deg(qq));
xlim(tlim);
xlabel('Time (s)');
ylim([-180, 180]);
ylabel('Degrees');
title('Optimized Trajectory');

figure(3);
plot(t, rad2deg(qq - qq0));
xlim(tlim);
xlabel('Time (s)');
ylabel('Degrees');
title('Trajectory Difference');

figure(4);
qVel = diff(rad2deg(qq));
qVel = [zeros(1, nq); qVel];
plot(t, abs(qVel));
xlim(tlim);
xlabel('Time (s)');
title('Speed');

drawnow;