%% Plot the joint trajectories in time
np = size(raw(1).qw0, 1);
nq = size(raw(1).qw0, 2);
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
% Optimization
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
% Optimization Difference
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
subplot(2,2,4);
t_opt = reshape([optimized.dt_opt], [3, numel(optimized)]);
bar(t_opt(1:1,:)');
xlim([1, n_optimizations]);
%legend('Solver', 'Setup+Solve');
xlabel('Iteration Number');
ylabel('Time (sec)');
title('Computation Solver Time');

%% Joint accelerations
figure(2);
% Original Plan
subplot(2,2,1);
qwAccel0 = diff(raw(1).qw0, 2);
qwAccel0 = qwAccel0 / (dt*dt);
plot(t(2:end-1), abs(qwAccel0));
xlim(tlim);
xlabel('Time (s)');
ylabel('Joint Accelerations (rad/s^2)');
title('Original Accelerations');
% q Optimization
subplot(2,2,2);
if kind==0
    qwAccel = diff(optimized(end).qw, 2);
else
    qwAccel = diff(optimized(end).qLambda, 2);
end
qwAccel = qwAccel / (dt*dt);
plot(t(2:end-1), abs(qwAccel));
xlim(tlim);
xlabel('Time (s)');
ylabel('Joint Accelerations (rad/s^2)');
if kind==0
    title('Optimized Accelerations');
else
    title('Optimized Accelerations (L)');
end
% Relative Acceleration against the base case
subplot(2,2,3);
relAccel = (abs(qwAccel) - abs(qwAccel0));% ./ qwAccel0;
plot(t(2:end-1), relAccel);
xlim(tlim);
xlabel('Time (s)');
ylabel('Joint Accelerations (rad/s^2)');
title('Acceleration Differences');
% Optimization function cost over time
subplot(2,2,4);
plot([optimized.opt_val]);
xlim([1, n_optimizations]);
xlabel('Iteration Number');
ylabel('Cost');
title('Optimization Costs');

% %% Task Space Target
% figure(3);
% 
% subplot(2,2,1);
% vw0pos0 = raw(1).vw0(2:end, 1:3);
% vw0rot0 = raw(1).vw0(2:end, 4:6);
% %%{
% vw0pos0 = vw0pos0 ./ repmat(sqrt(sum(vw0pos0 .^ 2, 2)), 1, size(vw0pos0, 2));
% vw0rot0 = vw0rot0 ./ repmat(sqrt(sum(vw0rot0 .^ 2, 2)), 1, size(vw0rot0, 2));
% %}
% 
% [hAx, hL1, hL2] = plotyy(...
%     t(2:end), vw0pos0, ...
%     t(2:end), vw0rot0);
% xlim(hAx(1), tlim);
% xlim(hAx(2), tlim);
% xlabel('Time (s)');
% ylabel(hAx(1), 'Translational (m/s)');
% ylabel(hAx(2), 'Angular (deg/s)');
% legend('x', 'y', 'z', 'Roll', 'Pitch', 'Yaw');
% title('Original Desired Task Velocity');
% 
% subplot(2,2,2);
% 
% vw0pos = raw(1).vw(2:end, 1:3);
% vw0rot = raw(1).vw(2:end, 4:6);
% %%{
% vw0pos = vw0pos ./ repmat(sqrt(sum(vw0pos .^ 2, 2)), 1, size(vw0pos, 2));
% vw0rot = vw0rot ./ repmat(sqrt(sum(vw0rot .^ 2, 2)), 1, size(vw0rot, 2));
% %}
% 
% [hAx, hL1, hL2] = plotyy(...
%     t(2:end), vw0pos, ...
%     t(2:end), vw0rot);
% xlim(hAx(1), tlim);
% xlim(hAx(2), tlim);
% xlabel('Time (s)');
% ylabel(hAx(1), 'Translational (m/s)');
% ylabel(hAx(2), 'Angular (deg/s)');
% legend('x', 'y', 'z', 'Roll', 'Pitch', 'Yaw');
% title('Original Task Velocity');
% 
% subplot(2,2,3);
% vwFpos = raw(end).vw(2:end, 1:3);
% vwFrot = raw(end).vw(2:end, 4:6);
% %%{
% vwFpos = vwFpos ./ repmat(sqrt(sum(vwFpos .^ 2, 2)), 1, size(vwFpos, 2));
% vwFrot = vwFrot ./ repmat(sqrt(sum(vwFrot .^ 2, 2)), 1, size(vwFrot, 2));
% %}
% 
% [hAx, hL1, hL2] = plotyy(...
%     t(2:end), vwFpos, ...
%     t(2:end), rad2deg(vwFrot));
% xlim(hAx(1), tlim);
% xlim(hAx(2), tlim);
% xlabel('Time (s)');
% ylabel(hAx(1), 'Translational (m/s)');
% ylabel(hAx(2), 'Angular (deg/s)');
% legend('x', 'y', 'z', 'Roll', 'Pitch', 'Yaw');
% title('Optimized Task Velocity');
% 
% %subplot(2,2,4);
% h_delta_task = figure(13);
% clf;
% [hAx, hL1, hL2] = plotyy(...
%     t(2:end), (raw(end).vw(2:end, 1:3) - raw(1).vw(2:end, 1:3)), ...
%     t(2:end), rad2deg(raw(end).vw(2:end, 4:6) - raw(1).vw(2:end, 4:6)));
% hL1(1).LineStyle = '--';
% hL1(1).LineWidth = 0.5;
% hL1(2).LineStyle = '--';
% hL1(2).LineWidth = 1;
% hL1(3).LineStyle = '--';
% hL1(3).LineWidth = 2;
% hL2(1).LineStyle = '-';
% hL2(1).LineWidth = 0.5;
% hL2(2).LineStyle = '-';
% hL2(2).LineWidth = 1;
% hL2(3).LineStyle = '-';
% hL2(3).LineWidth = 2;
% xlim(hAx(1), tlim);
% xlim(hAx(2), tlim);
% xlabel('Time (s)', 'FontSize', 12');
% ylabel(hAx(1), 'Translational (m/s)', 'FontSize', 12);
% ylabel(hAx(2), 'Angular (deg/s)', 'FontSize', 12);
% h_leg = legend('x', 'y', 'z', 'Roll', 'Pitch', 'Yaw');
% set(h_leg, 'FontSize', 10');
% if kind==0
%     title('Change in Task Velocity', 'FontSize', 16);
% else
%     title('Change in Task Velocity (L)', 'FontSize', 16);
% end
% 
% %% Change in the preference
% h_delta_pref = figure(14);
% plot(t, rad2deg(raw(end).qw0 - repmat(qwStar, [np,1])));
% xlim(tlim);
% ylim([-110, 110]);
% xlabel('Time (s)', 'FontSize', 12);
% ylabel('Joint Angle (deg)', 'FontSize', 12);
% if kind==0
%     title('Difference from q_H', 'FontSize', 16);
% else
%     title('Difference from q_H (L)', 'FontSize', 16);
% end
% 
% %% Change in the preference
% h_delta_pref = figure(24);
% %plot(t, abs(angdiff(raw(end).qw0, repmat(qwStar, [np,1]))/angdiff(raw(1).qw0, repmat(qwStar, [np,1]))));
% %plot(t, rad2deg(angdiff(raw(1).qw0, repmat(qwStar, [np,1]))));
% %plot(t, rad2deg(angdiff(raw(end).qw0, repmat(qwStar, [np,1]))));
% %ylim([-90, 90]);
% plot(t, ...
%     abs(rad2deg(angdiff(raw(end).qw0, repmat(qwStar, [np,1])))) ...
%     - abs(rad2deg(angdiff(raw(1).qw0, repmat(qwStar, [np,1])))) ...
%     );
% 
% xlim(tlim);
% 
% xlabel('Time (s)', 'FontSize', 12);
% ylabel('Joint Angle (deg)', 'FontSize', 12);
% if kind==0
%     title('Difference from q_H', 'FontSize', 16);
% else
%     title('Difference from q_H (L)', 'FontSize', 16);
% end

% %% Optimized
% h_optimal = figure(15);
% plot(t, rad2deg(raw(end).qw0));
% xlim(tlim);
% ylim([-180, 180]);
% xlabel('Time (s)', 'FontSize', 12);
% ylabel('Position (deg)', 'FontSize', 12);
% if kind==0
%     title('Optimized Trajectory', 'FontSize', 16);
% else
%     title('Optimized Trajectory (L)', 'FontSize', 16);
% end
% 
% %% Original
% h_orig = figure(16);
% plot(t, rad2deg(raw(1).qw0));
% xlim(tlim);
% ylim([-180, 180]);
% xlabel('Time (s)', 'FontSize', 12);
% ylabel('Position (deg)', 'FontSize', 12);
% if kind==0
%     title('Original Trajectory', 'FontSize', 16);
% else
%     title('Original Trajectory (L)', 'FontSize', 16);
% end

%% Save
% if kind==0
%     figure(h_delta_task);
%     set(h_delta_task,'PaperPositionMode','Auto') ;
%     print('delta_task_q', '-dpng');
%     %
%     figure(h_delta_pref);
%     set(h_delta_pref,'PaperPositionMode','Auto');
%     print('delta_pref_q', '-dpng');
%     %
%     figure(h_optimal);
%     set(h_optimal,'PaperPositionMode','Auto');
%     print('optimal_q', '-dpng');
%     %
%     figure(h_orig);
%     set(h_optimal,'PaperPositionMode','Auto');
%     print('original_q', '-dpng');
% else
% %     figure(h_delta_task);
% %     set(h_delta_task,'PaperPositionMode','Auto');
% %     print('delta_task_lambda', '-dpng');
%     %
% %     figure(h_delta_pref);
% %     set(h_delta_pref,'PaperPositionMode','Auto');
% %     print('delta_pref_lambda', '-dpng');
%     %
% %     figure(h_optimal);
% %     set(h_optimal,'PaperPositionMode','Auto');
% %     print('optimal_lambda', '-dpng');
% end

%% Calculations
% Against human q
dqh0 = sum(sum(angdiff(raw(1).qw0, repmat(qwStar, [np,1])).^2, 2))
dqhF = sum(sum(angdiff(raw(end).qw0, repmat(qwStar, [np,1])).^2, 2))

% Task length
dT0 = sum( sum( (raw(1).vw).^2, 2) )
dTF = sum( sum( (raw(end).vw).^2, 2) )

%sum(sum( diff(raw(1).vw(:, 1:3)).^2, 2 ))

%% Draw
drawnow;