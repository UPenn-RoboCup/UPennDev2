%% Clear the environment
clear all;
close all;
startup;

%% Figures
% Colormap for the labeled image
cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
cmap = [cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];
figure(1);
clf;
% (top)
% LabelA
f_lA = subplot(1,2,1);
im_lA = image(zeros(1));
colormap(cmap);
% yuyv
f_yuyv = subplot(1,2,2);
im_yuyv = image(zeros(1));
hold on;
p_ball = plot([0],[0],'m*');
hold off;
drawnow;

% Save the camera handles
cam = {};
cam.f_lA = f_lA;
cam.im_lA = im_lA;
cam.f_yuyv = f_yuyv;
cam.im_yuyv = im_yuyv;
cam.p_ball = p_ball;
% Plot scale
% Default: labelA is half size, so scale twice
scale = 2;
cam.scale = 2;

%% Network
% Add the UDP network
fd = udp_recv('new', 33333);
s_top = zmq('fd', fd);

%% Loop
running = 1;
while running
    % 1 second timeout
    idx = zmq('poll', 1000);
    for s=1:numel(idx)
        s_idx = idx(s);
        s_idx_m = s_idx + 1;
        [s_data, has_more] = zmq( 'receive', s_idx );
        while udp_recv('getQueueSize', fd) > 0
            udp_data = udp_recv('receive',fd);
        end
        [metadata, offset] = msgpack('unpack', udp_data);
        % This must be uint8
        raw = udp_data(offset+1:end);
        is_draw = process_libVision_msg(metadata, raw, cam);
        if is_draw==1
            drawnow;
        end
    end
end
