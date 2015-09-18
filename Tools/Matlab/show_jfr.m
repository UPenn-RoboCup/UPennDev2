%% ittybitty0 camera bytes
load udp_ittybitty0_trial1;
% Filter the time
trange = (b_in > 0);
ts_ittybitty0_in = t_in(trange);
bs_ittybitty0_in = b_in(trange);
dts_ittybitty0_in = diff(ts_ittybitty0_in);
dts_ittybitty0_in = dts_ittybitty0_in(dts_ittybitty0_in<10);
clear trange t_in b_in t_out b_out;

%% ittybitty1 camera bytes
load udp_ittybitty1_trial1;
% Filter the time
trange = (b_in > 0);
ts_ittybitty1_in = t_in(trange);
bs_ittybitty1_in = b_in(trange);
dts_ittybitty1_in = diff(ts_ittybitty1_in);
dts_ittybitty1_in = dts_ittybitty1_in(dts_ittybitty1_in<10);
clear trange t_in b_in t_out b_out;

%% feedback bytes
load udp_feedback_trial1;
% Filter the time
if exist('t_in', 'var')
    trange = (b_in > 0);
    ts_feedback_in = t_in(trange);
    bs_feedback_in = b_in(trange);
    dts_feedback_in = diff(ts_feedback_in);
    dts_feedback_in = dts_feedback_in(dts_feedback_in<10);
end
if exist('t_out', 'var')
    ts_feedback_out = t_out(trange);
    bs_feedback_out = b_out(trange);
    dts_feedback_out = diff(ts_feedback_out);
    dts_feedback_out = dts_feedback_out(dts_feedback_out<10);
end
clear trange t_in b_in t_out b_out;

%% mesh0 bytes
load udp_mesh0_trial1;
% Filter the time
if exist('t_in', 'var')
    trange = (b_in > 0);
    ts_mesh0_in = t_in(trange);
    bs_mesh0_in = b_in(trange);
end
if exist('t_out', 'var')
    trange = (b_out > 0);
    ts_mesh0_out = t_out(trange);
    bs_mesh0_out = b_out(trange);
end
clear trange t_in b_in t_out b_out;

%% mesh1 bytes
load udp_mesh0_trial1;
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
load udp_camera0_trial1;
% Filter the time
if exist('t_in', 'var')
    trange = (b_in > 0);
    ts_camera0_in = t_in(trange);
    bs_camera0_in = b_in(trange);
end
if exist('t_out', 'var')
    trange = (b_out > 0);
    ts_camera0_out = t_out(trange);
    bs_camera0_out = b_out(trange);
end
clear trange t_in b_in t_out b_out;

%% camera1 bytes
load udp_camera0_trial1;
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

%% Plot everything
figure(1);
plot(ts_ittybitty0_in, bs_ittybitty0_in, 'rx', ...
    ts_ittybitty1_in, bs_ittybitty1_in, 'bo', ...
    ts_feedback_in, bs_feedback_in, 'k-');
xlim([...
    min([ts_ittybitty0_in(1), ts_ittybitty1_in(1), ts_feedback_in(1)]), ...
    max([ts_ittybitty0_in(end), ts_ittybitty1_in(end), ts_feedback_in(end)])...
    ]);
title('Inside Transmission', 'FontSize', 16);
xlabel('Time since trial start (sec)', 'FontSize', 14);
ylabel('Bytes Transmitted', 'FontSize', 14);
legend('Head','Hand','Feedback');
%
figure(2);
histogram(dts_ittybitty0_in);
title('Distribution of Packet Intervals (Head Camera)', 'FontSize', 16);
xlabel('Successive Packet Interval (sec)', 'FontSize', 14);
figure(3);
histogram(dts_ittybitty1_in);
title('Distribution of Packet Intervals (Hand Camera)', 'FontSize', 16);
xlabel('Successive Packet Interval (sec)', 'FontSize', 14);
figure(4);
histogram(dts_feedback_in);
title('Distribution of Packet Intervals (Feedback)', 'FontSize', 16);
xlabel('Successive Packet Interval (sec)', 'FontSize', 14);
