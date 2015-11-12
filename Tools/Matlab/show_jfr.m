%% ittybitty0 camera bytes
load udp_ittybitty0_trial2;
% Filter the time
trange = (b_in > 0);
ts_ittybitty0_in = t_in(trange);
bs_ittybitty0_in = b_in(trange);
dts_ittybitty0_in = diff(ts_ittybitty0_in);
dts_ittybitty0_in = dts_ittybitty0_in(dts_ittybitty0_in<10);
clear trange t_in b_in t_out b_out;

%% ittybitty1 camera bytes
load udp_ittybitty1_trial2;
if exist('t_in', 'var')
    trange = (b_in > 0);
    ts_ittybitty1_in = t_in(trange);
    bs_ittybitty1_in = b_in(trange);
    dts_ittybitty1_in = diff(ts_ittybitty1_in);
    dts_ittybitty1_in = dts_ittybitty1_in(dts_ittybitty1_in<10);
end
if exist('t_out', 'var')
    trange = (b_out > 0);
    ts_ittybitty1_out = t_out(trange);
    bs_ittybitty1_out = b_out(trange);
    dts_ittybitty1_out = diff(ts_ittybitty1_out);
    dts_ittybitty1_out = dts_ittybitty1_out(dts_ittybitty1_out<10);
end
clear trange t_in b_in t_out b_out;

%% feedback bytes
load udp_feedback_trial2;
if exist('t_in', 'var')
    trange = (b_in > 0);
    ts_feedback_in = t_in(trange);
    bs_feedback_in = b_in(trange);
    dts_feedback_in = diff(ts_feedback_in);
    dts_feedback_in = dts_feedback_in(dts_feedback_in<10);
end
if exist('t_out', 'var')
    trange = (b_out > 0);
    ts_feedback_out = t_out(trange);
    bs_feedback_out = b_out(trange);
    dts_feedback_out = diff(ts_feedback_out);
    dts_feedback_out = dts_feedback_out(dts_feedback_out<10);
end
clear trange t_in b_in t_out b_out;

%% mesh0 bytes
load udp_mesh0_trial2;
% Filter the time
if exist('t_in', 'var')
    trange = (b_in > 0);
    ts_mesh0_in = t_in(trange);
    bs_mesh0_in = b_in(trange);
    dts_mesh0_in = diff(ts_mesh0_in);
    dts_mesh0_in = dts_mesh0_in(dts_mesh0_in<10);
end
if exist('t_out', 'var')
    trange = (b_out > 0);
    ts_mesh0_out = t_out(trange);
    bs_mesh0_out = b_out(trange);
    dts_mesh0_out = diff(ts_mesh0_out);
    dts_mesh0_out = dts_mesh0_out(dts_mesh0_out<10);
end
clear trange t_in b_in t_out b_out;

%% mesh1 bytes
load udp_mesh0_trial2;
% Filter the time
if exist('t_in', 'var')
    trange = (b_in > 0);
    ts_mesh1_in = t_in(trange);
    bs_mesh1_in = b_in(trange);
end
if exist('t_out', 'var')
    trange = (b_out > 0);
    ts_mesh1_out = t_out(trange);
    bs_mesh1_out = b_out(trange);
end
clear trange t_in b_in t_out b_out;

%% camera0 bytes
load udp_camera0_trial2;
% Filter the time
if exist('t_in', 'var')
    trange = (b_in > 0);
    ts_camera0_in = t_in(trange);
    bs_camera0_in = b_in(trange);
    dts_camera0_in = [0; diff(ts_camera0_in)];
end
if exist('t_out', 'var')
    trange = (b_out > 0);
    ts_camera0_out = t_out(trange);
    bs_camera0_out = b_out(trange);
    dts_camera0_out = diff(ts_camera0_out);
end
clear trange t_in b_in t_out b_out;

%% camera1 bytes
load udp_camera0_trial2;
% Filter the time
if exist('t_in', 'var')
    trange = (b_in > 0);
    ts_camera1_in = t_in(trange);
    bs_camera1_in = b_in(trange);
end
if exist('t_out', 'var')
    trange = (b_out > 0);
    ts_camera1_out = t_out(trange);
    bs_camera1_out = b_out(trange);
end
clear trange t_in b_in t_out b_out;

%% Node sent bytes
load node_trial2;
if exist('t_in', 'var')
    trange = (b_in > 0);
    ts_node_in = t_in(trange) / 1e3;
    bs_node_in = b_in(trange);
    dts_node_in = [0; diff(ts_node_in)];
    %dts_node_in = dts_node_in(dts_node_in>1e-2);
end
if exist('t_out', 'var')
    trange = (b_out > 0);
    ts_node_out = t_out(trange) / 1e3;
    bs_node_out = b_out(trange);
    dts_node_out = [0; diff(ts_node_out)];
    %dts_node_out = dts_node_out(dts_node_out>1e-2);
end
clear trange t_in b_in t_out b_out;

%% Timing
trange = [...
    min([ts_ittybitty0_in(1), ts_ittybitty1_in(1), ts_feedback_in(1), ts_node_in(1)]), ...
    max([ts_ittybitty0_in(end), ts_ittybitty1_in(end), ts_feedback_in(end), ts_node_in(end)]) ...
    ];

trange0 = [...
    min([ts_feedback_out(1), ts_node_out(1)]), ...
    max([ts_ittybitty0_in(end), ts_ittybitty1_in(end), ts_feedback_in(end), ts_node_in(end)]) ...
    ];

%% Slow Channel
figure(1);
plot(ts_ittybitty0_in - trange(1), bs_ittybitty0_in, 'rx', ...
    ts_ittybitty1_in - trange(1), bs_ittybitty1_in, 'bo', ...
    ts_feedback_in - trange(1), bs_feedback_in, 'k.', ...
    ts_node_in - trange(1), bs_node_in, 'gs' ...
    );
xlim(trange - trange(1));
title('Low Bandwidth Indoors Communication', 'FontSize', 16);
xlabel('Time Indoors (sec)', 'FontSize', 14);
ylabel('Bytes Transmitted', 'FontSize', 14);
legend('Head', 'Hand', 'Feedback', 'Commands');
print('drc-sdstream', '-dpng');

%% Fast Channel Indoors
figure(2);
[h, a1, a2] = plotyy(ts_camera0_in - trange(1), bs_camera0_in/1e3, ...
    ts_mesh0_in - trange(1), bs_mesh0_in/1e3);
title('High Bandwidth Indoors Communication', 'FontSize', 16);
xlim(trange - trange(1));
xlabel('Time Indoors (sec)', 'FontSize', 14);
ylabel(h(1), 'Camera Transmission (kB)', 'FontSize', 14);
ylabel(h(2), 'Mesh Transmission (kB)', 'FontSize', 14);
legend('Camera','Mesh');
a1.LineStyle = ':';
a1.Marker = 'x';
a2.LineStyle = ':';
a2.Marker = 'o';
print('drc-hdstream', '-dpng');

%% Network easing
figure(3);
plot(ts_camera0_in(dts_camera0_in>=1) - ts_camera0_in(1), dts_camera0_in(dts_camera0_in>=1));
title('Camera Stream Intervals over Time', 'FontSize', 16);
xlabel('Time Indoors (sec)', 'FontSize', 14);
ylabel('Transmission Interval (sec)', 'FontSize', 14);
xlim(trange - trange(1));
print('drc-hdinterval', '-dpng');

%% Slow Channel Packet Intervals
figure(4);
histogram(dts_ittybitty0_in);
title('Distribution of Packet Intervals (Head Camera)', 'FontSize', 16);
ylabel('Count');
xlabel('Successive Packet Interval (sec)', 'FontSize', 14);
print('drc-head-dist', '-dpng');
figure(5);
histogram(dts_ittybitty1_in);
title('Distribution of Packet Intervals (Hand Camera)', 'FontSize', 16);
ylabel('Count');
xlabel('Successive Packet Interval (sec)', 'FontSize', 14);
print('drc-hand-dist', '-dpng');
figure(6);
histogram(dts_feedback_in);
title('Distribution of Packet Intervals (Feedback)', 'FontSize', 16);
ylabel('Count');
xlabel('Successive Packet Interval (sec)', 'FontSize', 14);
print('drc-feedback-dist', '-dpng');
figure(5);
histogram(dts_camera0_in);
ylabel('Count');
title('Distribution of Packet Intervals (Camera)', 'FontSize', 16);
xlabel('Successive Packet Interval (sec)', 'FontSize', 14);
print('drc-camera-dist', '-dpng');

%% User interaction
figure(6);
histogram(dts_node_out);
title('Operator Command Rate (Outdoor)', 'FontSize', 16);
ylabel('Count', 'FontSize', 14);
xlabel('Packet Interval (sec)', 'FontSize', 14);
print('drc-command-diff-out', '-dpng');

figure(7);
histogram(dts_node_in);
title('Operator Command Rate (Indoor)', 'FontSize', 16);
ylabel('Count', 'FontSize', 14);
xlabel('Packet Interval (sec)', 'FontSize', 14);
print('drc-command-diff-in', '-dpng');

%%
figure(8);
total_sent = sum(bs_node_out) + sum(bs_node_in);
plot(ts_node_out - trange0(1), ...
    cumsum(bs_node_out), 'rx-', ...
    ts_node_in - trange0(1), ...
    cumsum(bs_node_in) + sum(bs_node_out), 'bo-');
xlim(trange0 - trange0(1));
ylim([0, total_sent]);
line(...
    trange0 - trange0(1), ...
    [sum(bs_node_out), sum(bs_node_out)], ...
    'Color', 'k', ...
    'LineWidth', 2, ...
    'LineStyle', '--' ...
    );
line(...
    [ts_node_in(1)+ts_node_out(end), ts_node_in(1) + ts_node_out(end)] / 2 - trange0(1), ...
    [0, total_sent], ...
    'Color', 'k', ...
    'LineWidth', 2, ...
    'LineStyle', ':' ...
    );
text(...
    sum(trange0 - trange0(1)) / 3, sum(bs_node_out)+196, ...
    sprintf('Indoors: %d bytes in %g seconds', sum(bs_node_in), ts_node_in(end) - ts_node_out(end)), ...
    'FontSize', 14 ...
    );
text(...
    sum(trange0 - trange0(1)) / 3, sum(bs_node_out)-196, ...
    sprintf('Outdoors: %d bytes in %g seconds', sum(bs_node_out), ts_node_out(end) - trange0(1)), ...
    'FontSize', 14 ...
    );
title('Command Bandwidth over Time', 'FontSize', 16);
xlabel('Operator to Robot Packet Interval (sec)', 'FontSize', 14);
ylabel('Cumulative Channel Usage (bytes)', 'FontSize', 14);
legend({'Indoors', 'Outdoors'}, 'Location', 'southeast', 'FontSize', 12);
print('drc-command-cumsum', '-dpng');