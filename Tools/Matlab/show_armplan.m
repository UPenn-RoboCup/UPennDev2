%% Plot the joint trajectories in time
np = numel(nulls);
nt = dt * np;
t = 0:dt:nt-dt;
tlim = [t(1), t(end)];

figure(1);
plot(t, rad2deg(qwPath0));
xlim(tlim);
xlabel('Time (s)');
ylim([-180, 180]);
ylabel('Degrees');
title('Original Trajectory');

if kind==0
    figure(2);
    plot(t, rad2deg(q));
    xlim(tlim);
    xlabel('Time (s)');
    ylim([-180, 180]);
    ylabel('Position (Degrees)');
    title('Optimized Trajectory');
    figure(3);
    plot(t, rad2deg(q - qwPath0));
    xlim(tlim);
    xlabel('Time (s)');
    ylabel('Position (Degrees)');
    title('Trajectory Difference');

    figure(4);
    qqVel = diff(q) / dt;
    qqVel = [zeros(1, nq); qqVel];
    plot(t, abs(qqVel));
    xlim(tlim);
    xlabel('Time (s)');
    ylabel('Speed (deg/s)');
    title('Joint Speeds');

    figure(5);
    qqAccel = diff(q, 2);
    qqAccel = [zeros(2, nq); qqAccel];
    plot(t, abs(qqAccel));
    xlim(tlim);
    xlabel('Time (s)');
    ylabel('Acceleration (deg/s^2)');
    title('Joint Accelerations');
end

%% Plot the Singular Values
figure(6);plot(t, Ss);xlim(tlim);title('Singular Values');xlabel('Time (s)');
%figure(7);plot(t,Vs);xlim(tlim);title('Vs');xlabel('Time (s)');
%figure(8);plot(t,Us);xlim(tlim);title('Us');xlabel('Time (s)');

%% Show the null space optimization result
if exist('lambda', 'var')
    figure(12);
    clf;
    hold on;
    cmap = hsv(numel(swapidx)-1);
    for i=1:numel(swapidx)-1
        range = swapidx(i):swapidx(i+1);
        for iN=1:nNull
            plot(t(range), lambda(range, iN), '-s', 'Color', cmap(i,:));
        end
    end
    hold off;
    xlim(tlim);
    xlabel('Time (s)');
    title('Original lambda [MATLAB]');
end

if exist('dlambda', 'var')
    figure(13);
    clf;
    hold on;
    cmap = hsv(numel(swapidx)-1);
    for i=1:numel(swapidx)-1
        range = swapidx(i):swapidx(i+1);
        for iN=1:nNull
            plot(t(range), dlambda(range, iN), '-s', 'Color', cmap(i,:));
        end
    end
    hold off;
    xlim(tlim);
    xlabel('Time (s)');
    title('Optimized dlambda');
end

if exist('ddlambda', 'var')
    figure(14);
    clf;
    hold on;
    cmap = hsv(numel(swapidx)-1);
    for i=1:numel(swapidx)-1
        range = swapidx(i):swapidx(i+1);
        for iN=1:nNull
            plot(t(range), ddlambda(range, iN), '-s', 'Color', cmap(i,:));
        end
    end
    hold off;
    xlim(tlim);
    xlabel('Time (s)');
    title('Optimized ddlambda');
end

if exist('qLambda', 'var')
    figure(15);
    plot(t, rad2deg(qLambda));
    xlim(tlim);
    ylim([-180, 180]);
    xlabel('Time (s)');
    title('Optimized Trajectory (lambda)');
end

if exist('dqLambda', 'var')
    figure(16);
    plot(t, rad2deg(dqLambda));
    xlim(tlim);
    xlabel('Time (s)');
    title('Angle change from lambda');
end

%% Draw
drawnow;