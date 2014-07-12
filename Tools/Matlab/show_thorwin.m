% Clear the environment
clear all;
close all;
startup;

%% Camera Figure


global cam monitor

monitor = show_monitor_thorwin();
monitor.init();




%% Network
% Add the UDP network
fd = udp_recv('new', 33333);
s_top = zmq('fd', fd);

%% Loop
running = 1;

tic;
t0 = toc;
count_world = 0;
count_detect = 0;
count_labelA = 0;
count_cam = 0;

t_last = toc;

data_yuyv=[];
data_yuyv.meta = [];
data_yuyv.raw = [];

data_world=[];
data_world.meta = [];
data_world.raw = [];

data_detect=[];
data_detect.meta = [];
data_detect.raw = [];

data_labelA=[];
data_labelA.meta = [];
data_labelA.raw = [];

last_draw_time = toc;
last_draw_duration = 0;

data_yuyv.recv = false;
data_world.recv = false;
data_detect.recv = false;
data_labelA.recv = false;

while running
    % 100ms timeout    
    wait_time = max(0, 0.066 - last_draw_duration);
    idx = zmq('poll', wait_time*1000);    
    recv_items = 0;
    for s=1:numel(idx)
        s_idx = idx(s);
        s_idx_m = s_idx + 1;
        [s_data, has_more] = zmq( 'receive', s_idx );
        while udp_recv('getQueueSize', fd) > 0
            recv_items = recv_items + 1;
            udp_data = udp_recv('receive',fd);
            [metadata, offset] = msgpack('unpack', udp_data);

            % This must be uint8
            raw = udp_data(offset+1:end);
            msg_id = char(metadata.id);
            if strcmp(msg_id,'world')
              count_world = count_world + 1;
              data_world.meta = metadata;
              data_world.raw = raw;
              data_world.recv = true;
            end
            if strcmp(msg_id,'detect')
              count_detect = count_detect + 1;
              data_detect.meta = metadata;
              data_detect.raw = raw;
              data_detect.recv = true;
            end
            if strcmp(msg_id,'labelA')
              count_labelA = count_labelA + 1;
              data_labelA.meta = metadata;
              data_labelA.raw = raw;
              data_labelA.recv = true;
            end
            if strcmp(msg_id,'head_camera')
              count_cam = count_cam + 1;
              data_yuyv.meta = metadata;
              data_yuyv.raw = raw;
              data_yuyv.recv = true;
            end            
        end
    end
    t=toc;
%    disp(sprintf('recv time: %.2f ms %d items',(t-t_last)*1000, recv_items));
    t_last = t;
    last_draw_duration = 0;

    if (t>last_draw_time+0.066) && (data_world.recv || data_detect.recv || data_labelA.recv) 

      if data_world.recv monitor.process_msg(data_world.meta,data_world.raw,cam); end
      if data_detect.recv monitor.process_msg(data_detect.meta,data_detect.raw,cam); end
      if data_labelA.recv monitor.process_msg(data_labelA.meta,data_labelA.raw,cam); end
      if data_yuyv.recv monitor.process_msg(data_yuyv.meta,data_yuyv.raw,cam); end
      drawnow; 
      t1=toc;
      last_draw_duration = t1-t_last;    
%      disp(sprintf('draw time: %.2f ms / %.2f fps',last_draw_duration*1000,1/(t-last_draw_time)));
      data_world.recv = false;
      data_detect.recv = false;
      data_labelA.recv = false;
      last_draw_time = t1;
    end
end
