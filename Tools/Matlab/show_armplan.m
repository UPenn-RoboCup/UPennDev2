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
nn = 1; % Number of null
gamma = 0.5;
Ns = cell(size(nulls));
Ns{1} = nulls{1};
for i=2:numel(nulls)
    Ns{i} = (1-gamma)*Ns{i-1} + gamma*nulls{i};
end

Ss = zeros(np, nq);
Vs = zeros(np, nq);
Us = zeros(np, nq);
lambda = zeros(np, nn);

REFLECTION_SIGN = 1;

for i=1:numel(nulls)
    %[U{i}, S{i}, V{i}] = svd(nulls{i});
    [U, S, V] = svd(Ns{i}');
    Vs(i,:) = V(:, 1);
    Us(i,:) = U(:, 1);
    
    %[U, S, V] = svd(gamma*nulls{i}'+(1-gamma)*nulls{i-1}');
    %[U, S, V] = svds(gamma*nulls{i}'+(1-gamma)*nulls{i-1}', 1);
    Ss(i,:) = diag(S);
    lambda(i,:) = S(1:nn,1:nn) * V(:, 1:nn)' * (qwPath{i} - qwPath{end});
    
end

swapidx = [1];
for i=2:numel(nulls)
    dirlambda = dot(Vs(i,:), Vs(i-1,:));
    if abs(dirlambda)>0.9
        % Prety much the same dir...
        if dirlambda < 0
            Vs(i,:) = -Vs(i,:);
            Us(i,:) = -Us(i,:);
            lambda(i,:) = -lambda(i,:);
        end
    else
        swapidx = [swapidx, i];
        fprintf('Dir switch: %d, %f\n', i, t(i));
    end
end
swapidx = [swapidx, numel(nulls)];

%% Plot them
figure(6);
plot(t, Ss);
xlim(tlim);
xlabel('Time (s)');
title('Singular Values');

figure(7);
clf;
hold on;
cmap = hsv(numel(swapidx)-1);
for i=1:numel(swapidx)-1
    range = swapidx(i):swapidx(i+1);
    plot(t(range), lambda(range), 'k-s', 'Color', cmap(i,:));
end

plot(t, lambda);
hold off;
xlim(tlim);
xlabel('Time (s)');
title('Original lambda [MATLAB]');

figure(10);plot(t,Vs);xlim(tlim);title('Vs');
figure(11);plot(t,Us);xlim(tlim);title('Us');

%% If lambda was given
if exist('dlamda0', 'var')
    dlambda0 = reshape(dlambda0, [nNull, np]);
    figure(11);
    plot(t, dlambda0);
    xlim(tlim);
    xlabel('Time (s)');
    title('Original lambda [torch]');
end
if kind==1
    dlambda = reshape(dlambda, [nNull, np]);
    figure(12);
    plot(t, dlambda);
    xlim(tlim);
    xlabel('Time (s)');
    title('Optimized lambda [torch]');
end

%% Draw
drawnow;