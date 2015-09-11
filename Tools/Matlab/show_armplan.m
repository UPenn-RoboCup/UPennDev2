%% Plot the joint trajectories in time
np = size(optimized(1).qw, 1);
nq = size(optimized(1).qw, 2);
nt = dt * np;
t = 0:dt:nt-dt;
tlim = [t(1), t(end)];

%% Joint trajectories
figure(1);
clf;
% Original Plan
subplot(2,2,1);
plot(t, rad2deg(raw(1).qw0));
xlim(tlim);
xlabel('Time (s)');
ylim([-180, 180]);
ylabel('Degrees');
title('Original Trajectory');
% Optimization results

% q Optimization
subplot(2,2,2);
plot(t, rad2deg(raw(end).qw0));
xlim(tlim);
xlabel('Time (s)');
ylim([-180, 180]);
ylabel('Position (deg)');

if kind==0
    title('Optimized Trajectory');
else
    title('Optimized (L) Trajectory');
end

% Difference
subplot(2,2,3);
plot(t, rad2deg(raw(end).qw0 - raw(1).qw0));
xlim(tlim);
xlabel('Time (s)');
ylabel('Difference (deg)');
if kind==0
    title('Trajectory Difference');
else
    title('Trajectory (L) Difference');
end


% TODO: Computation times...

%% Joint accelerations
figure(2);
% Original Plan
subplot(2,2,1);
qwAccel0 = diff(raw(1).qw0, 2) / (dt*dt);
qwAccel0 = [zeros(2, nq); qwAccel0];
plot(t, abs(qwAccel0));
xlim(tlim);
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
title('Original Accelerations');
% q Optimization
subplot(2,2,2);
qAccel = diff(optimized(end).qw, 2) / (dt*dt);
qAccel = [zeros(2, nq); qAccel];
plot(t, abs(qAccel));
xlim(tlim);
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
title('Optimized Accelerations');
% Lambda Optimization
subplot(2,2,3);
qLambdaAccel = diff(optimized(end).qLambda, 2) / (dt*dt);
qLambdaAccel = [zeros(2, nq); qLambdaAccel];
plot(t, abs(qLambdaAccel));
xlim(tlim);
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
title('Lambda Accelerations');
% TODO: Computation times...

%% Task Space Target
%%{
figure(30);
subplot(2,2,1);
[hAx, hL1, hL2] = plotyy(...
    t, rad2deg(raw(1).vw0(:, 1:3)), ...
    t, rad2deg(raw(1).vw0(:, 4:6)));
xlim(hAx(1), tlim);
xlim(hAx(2), tlim);
xlabel('Time (s)');
ylabel(hAx(1), 'Translational (m/s)');
ylabel(hAx(2), 'Angular (deg/s)');
legend('x', 'y', 'z', 'Roll', 'Pitch', 'Yaw');
title('Original Task Velocity');

subplot(2,2,2);
[hAx, hL1, hL2] = plotyy(...
    t, rad2deg(raw(end).vw0(:, 1:3)), ...
    t, rad2deg(raw(end).vw0(:, 4:6)));
xlim(hAx(1), tlim);
xlim(hAx(2), tlim);
xlabel('Time (s)');
ylabel(hAx(1), 'Translational (m/s)');
ylabel(hAx(2), 'Angular (deg/s)');
legend('x', 'y', 'z', 'Roll', 'Pitch', 'Yaw');
title('Optimized Task Velocity');

subplot(2,2,3);
[hAx, hL1, hL2] = plotyy(...
    t, rad2deg(raw(end).vw0(:, 1:3) - raw(1).vw0(:, 1:3)), ...
    t, rad2deg(raw(end).vw0(:, 4:6) - raw(1).vw0(:, 4:6)));
xlim(hAx(1), tlim);
xlim(hAx(2), tlim);
xlabel('Time (s)');
ylabel(hAx(1), 'Translational (m/s)');
ylabel(hAx(2), 'Angular (deg/s)');
legend('x', 'y', 'z', 'Roll', 'Pitch', 'Yaw');
title('Change in Task Velocity');

subplot(2,2,4);
%plot(t, raw(end).vw0 - raw(1).vw0);
%legend('x', 'y', 'z', 'Roll', 'Pitch', 'Yaw');
%title('Change in Task Velocity');
%}

%% Draw
drawnow;