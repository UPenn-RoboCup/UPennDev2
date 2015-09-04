%% Plot the joint trajectories in time
qq0 = rad2deg([qwPath{:}])';
%qq0 = reshape(rad2deg(qPath0), [nq, np])';
np = numel(nulls);
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

if kind==0
    figure(2);
    qq = reshape(rad2deg(q), [nq, np])';
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
end

%% Plot the singular values

Ss = zeros(np, numel(qArm0));
%U = {};
%S = {};
%V = {};
lambda = zeros(np, 1);
for i=1:numel(nulls)
    %[U{i}, S{i}, V{i}] = svd(nulls{i});
    [U, S, V] = svd(nulls{i}');
    Ss(i,:) = diag(S);
    lambda(i) = V(:, 1)' * (qwPath{i} - qwPath{end});
    if i>=47 && i<=50
        i
        U
        diag(S)'
        V
    end
    if i>=49
        lambda(i) = -1*lambda(i);
    end
end
figure(6);
plot(Ss);
title('Singular Values');

figure(7);
plot(lambda);
title('Original lambda [MATLAB]')

%% If lambda was given
if kind==1
    dlambda0 = reshape(dlambda0, [nNull, np]);
    dlambda = reshape(dlambda, [nNull, np]);
    figure(11);
    plot(dlambda0);
    title('Original lambda [torch]');
    
    figure(12);
    plot(dlambda);
    title('Optimized lambda [torch]');
end

%% Draw
drawnow;