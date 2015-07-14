close all;

logdir = strcat('Data/', 'lines3', '/');

% Grab the common metadata
fid = fopen(strcat(logdir,'count_d_m.log'));
meta = fread(fid, Inf,'*int8');
metadata = msgpack('unpack', meta);
fclose(fid);

% counts
fid = fopen(strcat(logdir,'count_d_r.log'));
cnt = fread(fid,Inf,'*int');
cnt = reshape(cnt, [metadata.MAXR, metadata.NTH]);
fclose(fid);
figure(1);
imagesc(cnt);

% Read the labelB image
fid = fopen(strcat(logdir,'labelB_d_r.log'));
labelB = fread(fid,Inf,'*uint8');
labelB = reshape(labelB, [metadata.wb, metadata.hb]);
fclose(fid);

% Plot labelB
cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
cmap = [cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];
figure(2);
image(labelB');
colormap(cmap);
hold on;
p_l = plot([0],[0],'m*-');

% Show the lines
propsLine = metadata;
l_x = double([propsLine.iMin, propsLine.iMean, metadata.iMax]);
l_y = double([propsLine.jMin, propsLine.jMean, metadata.jMax]);
set(p_l, 'Xdata', l_x );
set(p_l, 'Ydata', l_y );

