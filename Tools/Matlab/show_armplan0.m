%% Plot the joint trajectories in time
tlim = [t(1), t(end)];

if exist('qwPath0', 'var')
    figure(1);
    plot(t, rad2deg(qwPath0));
    xlim(tlim);
    xlabel('Time (s)');
    ylim([-180, 180]);
    ylabel('Degrees');
    title('Original Trajectory');
end

if exist('swapidx', 'var')
    cmap = hsv(numel(swapidx)-1);
end

if exist('q', 'var')
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
    for i=1:numel(swapidx)-1
        range = swapidx(i):swapidx(i+1);
        for iN=1:nNull
            plot(t(range), lambda(range, iN), '-s', 'Color', cmap(i,:)*(iN/nNull));
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
    for i=1:numel(swapidx)-1
        range = swapidx(i):swapidx(i+1);
        for iN=1:nNull
            plot(t(range), dlambda(range, iN), '-s', 'Color', cmap(i,:)*(iN/nNull));
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
    for i=1:numel(swapidx)-1
        range = swapidx(i):swapidx(i+1);
        for iN=1:nNull
            plot(t(range), ddlambda(range, iN), '-s', 'Color', cmap(i,:)*(iN/nNull));
        end
    end
    hold off;
    xlim(tlim);
    xlabel('Time (s)');
    title('Optimized ddlambda');
end

if exist('lambda0', 'var')
    figure(17);
    clf;
    cmap = hsv(numel(swapidx0)-1);
    hold on;
    for i=1:numel(swapidx0)-1
        range = swapidx0(i):swapidx0(i+1);
        for iN=1:nNull
            plot(t(range), lambda0(range, iN), '-s', 'Color', cmap(i,:)*(iN/nNull));
        end
    end
    hold off;
    xlim(tlim);
    xlabel('Time (s)');
    title('Original lambda');
end

if exist('dqLambda', 'var')
    figure(16);
    plot(t, rad2deg(dqLambda));
    xlim(tlim);
    xlabel('Time (s)');
    xlabel('(deg)');
    title('Trajectory Difference (lambda)');
end

if exist('qLambda', 'var')
    figure(15);
    plot(t, rad2deg(qLambda));
    xlim(tlim);
    ylim([-180, 180]);
    xlabel('Time (s)');
    title('Optimized Trajectory (lambda)');
    
    figure(18);
    qLVel = diff(qLambda) / dt;
    qLVel = [zeros(1, nq); qLVel];
    plot(t, abs(qLVel));
    xlim(tlim);
    xlabel('Time (s)');
    ylabel('Speed (rad/s)');
    title('Joint Speeds (lambda)');
    
    figure(19);
    qLAccel = diff(qLambda, 2);
    qLAccel = [zeros(2, nq); qLAccel];
    plot(t, abs(qLAccel));
    xlim(tlim);
    xlabel('Time (s)');
    ylabel('Acceleration (rad/s^2)');
    title('Joint Accelerations (lambda)');
end

%% Compare
if exist('qqAccel', 'var')
    figure(20);
    plot(t, abs(qLAccel) - qqAccel);
    xlim(tlim);
    xlabel('Time (s)');
    ylabel('Acceleration (deg/s^2)');
    title('Joint Accelerations (diff)');
end

if exist('qqVel', 'var')
    figure(21);
    plot(t, abs(qLVel) - abs(qqVel));
    xlim(tlim);
    xlabel('Time (s)');
    ylabel('Speed (rad/s)');
    title('Joint Speeds (diff)');
end

if exist('q', 'var')
    figure(22);
    plot(t, rad2deg(qLambda - q));
    xlim(tlim);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Joint Positions (diff)');
end

if exist('lambda', 'var')
    figure(23);
    plot(t, ddlambda - lambda);
    xlim(tlim);
    xlabel('Time (s)');
    ylabel('l');
    title('Lambda Positions (diff)');
end

%% Task Space Target
figure(30);
plot(t, vwPath00);
xlim(tlim);
legend('Roll', 'Pitch', 'Yaw', 'x', 'y', 'z');

figure(31);
plot(t, vwPath0);
xlim(tlim);
legend('Roll', 'Pitch', 'Yaw', 'x', 'y', 'z');

%% Draw
drawnow;