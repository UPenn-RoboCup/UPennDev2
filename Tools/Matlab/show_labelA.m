clear all;
% Colormap for the labeled image
cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
cmap = [cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];
figure(1);
clf;
% LabelA
subplot(2,2,1);
i_az_t = image(zeros(240,320));
colormap(cmap);
% yuyv
subplot(2,2,2);
i_j_t = image(zeros(480,640));
hold on;
p_j_t = plot([120],[160],'m*');
hold off;
% LabelA (bottom)
subplot(2,2,3);
i_az_b = image(zeros(120,160));
colormap(cmap);
% yuyv (bottom)
subplot(2,2,4);
i_j_b = image(zeros(240,320));
%
drawnow;

% Add the monitor
top_fd = udp_recv('new', 33333);
s_top = zmq( 'fd', top_fd );
bottom_fd  = udp_recv('new', 33334);
s_bottom = zmq( 'fd', bottom_fd );
% Cameras
cams = {};
cams{1} = {};
cams{1}.labelA = i_az_t;
cams{1}.yuyv = i_j_t;
cams{1}.fd = top_fd;
%
cams{2} = {};
cams{2}.labelA = i_az_b;
cams{2}.yuyv = i_j_b;
cams{2}.fd = bottom_fd;

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
        raw = udp_data(offset+1:end); % This must be uint8
        compr_t = char(metadata.c);
        if strcmp(compr_t,'jpeg')
            jimg = djpeg(raw);
            set(cam.yuyv,'Cdata', jimg);
        elseif strcmp(compr_t,'zlib')
            Az = reshape(zlibUncompress(raw),[metadata.w,metadata.h])';
            set(cam.labelA,'Cdata', Az);
            if isa(metadata.ball,'double')
                set(p_j_t,'Xdata', 2*metadata.ball(1));
                set(p_j_t,'Ydata', 2*metadata.ball(2));
            else
                set(p_j_t,'Xdata', 0);
                set(p_j_t,'Ydata', 0);
            end
        end
        drawnow;
    end
end