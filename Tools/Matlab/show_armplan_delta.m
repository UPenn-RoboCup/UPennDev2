%% Plot the joint trajectories in time
np = size(raw(1).qw0, 1);
nq = size(raw(1).qw0, 2);
nt = dt * np;
t = 0:dt:nt-dt;
tlim = [t(1), t(end)];

%% Original
h_orig = figure(1);
set(h_orig,'PaperPositionMode','Auto');
plot(t, rad2deg(raw(1).qw0));
xlim(tlim);
ylim([-180, 180]);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Position (deg)', 'FontSize', 12);
if kind==0
    title('Original Trajectory', 'FontSize', 16);
    print('optimal_q_0', '-dpng');
else
    title('Original Trajectory', 'FontSize', 16);
    print('optimal_lambda_0', '-dpng');
end


%% Optimized
for i=2:numel(raw)
    h_optimal = figure(i);
    plot(t, rad2deg(raw(i).qw0));
    set(h_optimal,'PaperPositionMode','Auto');
    xlim(tlim);
    ylim([-180, 180]);
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Position (deg)', 'FontSize', 12);
    if kind==0
        title(sprintf('Optimized Trajectory %d', i-1), 'FontSize', 16);
        print(sprintf('optimal_q_%d', i-1), '-dpng');
    else
        title(sprintf('Optimized Trajectory %d', i-1), 'FontSize', 16);
        print(sprintf('optimal_lambda_%d', i-1), '-dpng');
    end
end