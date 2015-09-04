%% Plot the joint trajectories in time
qq = reshape(rad2deg(q), [nq, np])';
qq0 = reshape(rad2deg(qPath0), [nq, np])';
nt = dt * np;
t = 0:dt:nt-dt;
tlim = [t(1), t(end)];

figure(1);
plot(t, qq0);
xlim(tlim);
xlabel('Time (s)');
ylim([-180, 180]);
ylabel('Degrees');
title('Original Trajectory');

figure(2);
plot(t, qq);
xlim(tlim);
xlabel('Time (s)');
ylim([-180, 180]);
ylabel('Position (Degrees)');
title('Optimized Trajectory');

figure(3);
plot(t, qq - qq0);
xlim(tlim);
xlabel('Time (s)');
ylabel('Position (Degrees)');
title('Trajectory Difference');

figure(4);
qqVel = diff(qq) / dt;
qqVel = [zeros(1, nq); qqVel];
plot(t, abs(qqVel));
xlim(tlim);
xlabel('Time (s)');
ylabel('Speed (deg/s)');
title('Joint Speeds');

figure(5);
qqAccel = diff(qq, 2);
qqAccel = [zeros(2, nq); qqAccel];
plot(t, abs(qqAccel));
xlim(tlim);
xlabel('Time (s)');
ylabel('Acceleration (deg/s^2)');
title('Joint Accelerations');

%% Plot the singular values
np = numel(nulls);
Ss = zeros(np, numel(qArm0));
%U = {};
%S = {};
%V = {};
lambda = zeros(np, 1);
for i=1:numel(nulls)
    %[U{i}, S{i}, V{i}] = svd(nulls{i});
    [U, S, V] = svd(nulls{i});
    Ss(i,:) = diag(S);
    lambda(i) = V(:, 1)' * (qwPath{i} - qwPath{end});
end
figure(6);
plot(Ss);
title('Singular Values');

figure(7);
plot(lambda);

%% Draw
drawnow;