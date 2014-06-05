clear all;
close all;
startup;

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

% Add the UDP network
fd = udp_recv('new', 33333);
s_top = zmq( 'fd', fd );

% Plot scale
% Default: labelA is half size, so scale twice
scale = 2;

while 1
    % 1 second timeout
    idx = zmq('poll',1000);
    for s=1:numel(idx)
        s_idx = idx(s);
        s_idx_m = s_idx + 1;
        [s_data, has_more] = zmq( 'receive', s_idx );
        while udp_recv('getQueueSize',fd) > 0
            udp_data = udp_recv('receive',fd);
        end
        [metadata,offset] = msgpack('unpack',udp_data);
        if numel(udp_data)==offset
            % Process ball data
            if isfield(metadata, 'detect')
                if ~isa(metadata.ball,'uint8')
                    ball_c = metadata.ball.centroid * scale;
                    set(p_ball,'Xdata', ball_c(1));
                    set(p_ball,'Ydata', ball_c(2));
                else
                    % Error message
                    debug_b = char(metadata.ball);
                    set(p_ball,'Xdata', []);
                    set(p_ball,'Ydata', []);
                end
            end
        else
            % Process the raw data
            raw = udp_data(offset+1:end); % This must be uint8
            compr_t = char(metadata.c);
            if strcmp(compr_t,'jpeg')
                set(im_yuyv,'Cdata', djpeg(raw));
                xlim(f_yuyv,[0 metadata.w]);
                ylim(f_yuyv,[0 metadata.h]);
            elseif strcmp(compr_t,'zlib')
                Az = reshape(zlibUncompress(raw),[metadata.w,metadata.h])';
                set(im_lA,'Cdata', Az);
                xlim(f_lA,[0 metadata.w]);
                ylim(f_lA,[0 metadata.h]);
            end
            drawnow;
        end
    end
end