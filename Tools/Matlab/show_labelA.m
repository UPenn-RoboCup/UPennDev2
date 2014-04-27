clear all;
close all;
startup;
% Camera data for MATLAB GUI
cams = {};
cams{1} = {};
cams{2} = {};

% Colormap for the labeled image
cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
cmap = [cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];
figure(1);
clf;
% (top)
% LabelA
cams{1}.f_lA = subplot(2,2,1);
cams{1}.im_lA = image(zeros(1));
colormap(cmap);
% yuyv
cams{1}.f_yuyv = subplot(2,2,2);
cams{1}.im_yuyv = image(zeros(1));
hold on;
cams{1}.p_ball = plot([0],[0],'m*');
hold off;
% (bottom)
% LabelA 
cams{2}.f_lA = subplot(2,2,3);
cams{2}.im_lA = image(zeros(1));
colormap(cmap);
% yuyv (bottom)
cams{2}.f_yuyv = subplot(2,2,4);
cams{2}.im_yuyv = image(zeros(1));
hold on;
cams{2}.p_ball = plot([0],[0],'m*');
hold off;
%
drawnow;

% Add the monitor
cams{1}.fd = udp_recv('new', 33333);
s_top = zmq( 'fd', cams{1}.fd );
cams{2}.fd = udp_recv('new', 33334);
s_bottom = zmq( 'fd', cams{2}.fd );

% Plot scale
% Default: labelA is half size, so scale twice
cams{1}.scale = 2;
cams{2}.scale = 2;

while 1
    % 1 second timeout
    idx = zmq('poll',1000);
    for s=1:numel(idx)
        s_idx = idx(s);
        s_idx_m = s_idx + 1;
        cam = cams{s_idx_m};
        [s_data, has_more] = zmq( 'receive', s_idx );
        while udp_recv('getQueueSize',cam.fd) > 0
            udp_data = udp_recv('receive',cam.fd);
        end
        [metadata,offset] = msgpack('unpack',udp_data);
        if numel(udp_data)==offset
            % Process ball data
            if isfield(metadata, 'detect')
                if ~isa(metadata.ball,'uint8')
                    ball_c = metadata.ball.centroid * cam.scale;
                    set(cam.p_ball,'Xdata', ball_c(1));
                    set(cam.p_ball,'Ydata', ball_c(2));
                else
                    % Error message
                    debug_b = char(metadata.ball);
                    set(cam.p_ball,'Xdata', []);
                    set(cam.p_ball,'Ydata', []);
                end
            end
        else
            % Process the raw data
            raw = udp_data(offset+1:end); % This must be uint8
            compr_t = char(metadata.c);
            if strcmp(compr_t,'jpeg')
                set(cam.im_yuyv,'Cdata', djpeg(raw));
                xlim(cam.f_yuyv,[0 metadata.w]);
                ylim(cam.f_yuyv,[0 metadata.h]);
            elseif strcmp(compr_t,'zlib')
                Az = reshape(zlibUncompress(raw),[metadata.w,metadata.h])';
                set(cam.im_lA,'Cdata', Az);
                xlim(cam.f_lA,[0 metadata.w]);
                ylim(cam.f_lA,[0 metadata.h]);
            end
            drawnow;
        end
    end
end