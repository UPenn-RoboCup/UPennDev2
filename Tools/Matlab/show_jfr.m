%% Times
t0      = 1433536200; % Start the car
tdoor   = 1433537480;
tinside = 1433538150;
tvalve  = 0;
t1      = 1433539775.267914; % Fall down

%% ittybitty0 camera bytes
load('/Users/smcgill3/UPennDev/Tools/Matlab/Data/udp_ittybitty0.mat');
% Filter the time
trange = (t >= tinside) & (bytes > 0);
ts_ittybitty0 = t(trange) - t0;
bs_ittybitty0 = bytes(trange);

%% ittybitty1 camera bytes
load('/Users/smcgill3/UPennDev/Tools/Matlab/Data/udp_ittybitty1.mat');
trange = t >= tinside;
ts_ittybitty1 = t(trange) - t0;
bs_ittybitty1 = bytes(trange);

%% feedback bytes
load('/Users/smcgill3/UPennDev/Tools/Matlab/Data/udp_feedback.mat');
trange = t >= tinside;
ts_feedback = t(trange) - t0;
bs_feedback = bytes(trange);

%% Clear and prepare to save
clear t bytes;
save('Data/network_data.mat');

%% Plot everything
% figure(1);
% plot(ts_ittybitty0, bs_ittybitty0, 'x');
% xlim([tinside, t1] - t0);
% title('Low Resolution Head', 'FontSize', 16);
% xlabel('Time since trial start (sec)', 'FontSize', 14);
% ylabel('Bytes Transmitted', 'FontSize', 14);
% 
% figure(2);
% plot(ts_ittybitty1, bs_ittybitty1);
% xlim([tinside, t1] - t0);
% title('Low Resolution Hand', 'FontSize', 16);
% xlabel('Time since trial start (sec)', 'FontSize', 14);
% ylabel('Bytes Transmitted', 'FontSize', 14);
% 
% figure(3);
% plot(ts_feedback, bs_feedback);
% xlim([tinside, t1] - t0);
% title('Feedback', 'FontSize', 16);
% xlabel('Time since trial start (sec)', 'FontSize', 14);
% ylabel('Bytes Transmitted', 'FontSize', 14);

figure(1);
plot(ts_ittybitty0, bs_ittybitty0, 'rx', ...
    ts_ittybitty1, bs_ittybitty1, 'bo', ...
    ts_feedback, bs_feedback, '.');
xlim([tinside, t1] - t0);
title('Inside Transmission', 'FontSize', 16);
xlabel('Time since trial start (sec)', 'FontSize', 14);
ylabel('Bytes Transmitted', 'FontSize', 14);
legend('Head','Hand','Feedback');
