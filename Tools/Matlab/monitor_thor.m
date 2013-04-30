clear all;
recv_fd = udp_recv();
s_udp = zmq( 'fd', recv_fd );

w=640;
h=480;
udp_img = zeros(w,h,3);

f = figure(1);
set(gcf,'doublebuffer','off');
subplot(1,2,1);
pc = gca;
hc = imagesc( udp_img );
xlim([1 w]);
ylim([1 h]);
drawnow;

%% Timing
t0 = tic;
target_fps = 30;
inv_target_fps = 1/target_fps;
dirty = 0;
counter = uint32(0);

while 1
    counter = counter+1;
    [data,idx] = zmq('poll',1000);
    for s=1:numel(idx)
        if idx(s)==s_udp
            while udp_recv('getQueueSize') > 0
                udp_data = udp_recv('receive');
            end
            udp_img = djpeg(udp_data);
            set(hc,'Cdata',udp_img);
            dirty = 1;
        end
    end
    % Control the drawing
    t_diff = toc(t0);
    if t_diff>inv_target_fps
        fprintf('FPS: %.1f\n',1/t_diff);
        if dirty==1
            title(pc, sprintf('FPS: %.1f',1/t_diff) );
            dirty = 0;
            drawnow;
        end
        counter = 0;
        t0 = tic;
    end
end