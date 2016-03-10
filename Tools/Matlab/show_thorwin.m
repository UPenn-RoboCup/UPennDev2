% Clear the environment
%clear all;
%close all;
%startup;

%% Camera Figure


global cam monitor

monitor = show_monitor_thorwin();
monitor.init();


%% Network
s_camera = zmq('subscribe', 'ipc', 'camera0');
s_vision = zmq('subscribe', 'ipc', 'vision0');
s_label = zmq('subscribe', 'ipc', 'label');
%s_world = zmq('subscribe', 'ipc', 'world');

p0 = zmq('publish', 'ipc', 'rrt1');

% Add the UDP network
f_camera = udp_recv('new', 17003);
f_vision = udp_recv('new', 17013);
f_label = udp_recv('new', 17014);
f_world = udp_recv('new', 17023);
%
s_camera_udp = zmq('fd', f_camera);
s_vision_udp = zmq('fd', f_vision);
s_label_udp = zmq('fd', f_label);
s_world_udp = zmq('fd', f_world);

%% Loop
running = 1;

tic;
t0 = toc;
count_world = 0;
count_detect = 0;
count_labelA = 0;
count_labelB = 0;
count_cam = 0;

t_last = toc;

data_yuyv=[];
data_yuyv.meta = [];
data_yuyv.raw = [];

data_world=[];
data_world.meta = [];
%data_world.raw = [];

data_detect=[];
data_detect.meta = [];
%data_detect.raw = [];

data_labelA=[];
data_labelA.meta = [];
data_labelA.raw = [];

data_labelB=[];
data_labelB.meta = [];
data_labelB.raw = [];

last_draw_time = toc;
last_draw_duration = 0;

data_yuyv.recv = false;
data_world.recv = false;
data_detect.recv = false;
data_labelA.recv = false;
data_labelB.recv = false;
run_time = 0;
while run_time < 1000 %running
    run_time = run_time + 1;
    % 100ms timeout
    wait_time = max(0, 0.066 - last_draw_duration);
    idx = zmq('poll', wait_time*1000);
    recv_items = 0;
    for s=1:numel(idx)
        s_idx = idx(s);
        s_idx_m = s_idx + 1;
        [meta, has_more] = zmq( 'receive', s_idx );
        [metadata, offset] = msgpack('unpack', meta);
        %disp(metadata);
        msg_id = 'unknown';
        if isstruct(metadata)
            msg_id = char(metadata.id);
            if has_more>0
                [raw, has_more] = zmq( 'receive', s_idx );
            end
        end
        if strcmp(msg_id,'world')
            count_world = count_world + 1;
            data_world.meta = metadata;
            %data_world.raw = raw;
            data_world.recv = true;
        end
        if strcmp(msg_id,'detect')
            count_detect = count_detect + 1;
            data_detect.meta = metadata;
            %data_detect.raw = raw;
            data_detect.recv = true;
        end
        if strcmp(msg_id,'labelA')
            count_labelA = count_labelA + 1;
            data_labelA.meta = metadata;
            data_labelA.raw = raw;
            data_labelA.recv = true;
        end
        if strcmp(msg_id,'labelB')
            count_labelB = count_labelB + 1;
            data_labelB.meta = metadata;
            data_labelB.raw = raw;
            data_labelB.recv = true;
        end
        if strcmp(msg_id,'head_camera')
            count_cam = count_cam + 1;
            data_yuyv.meta = metadata;
            data_yuyv.raw = raw;
            data_yuyv.recv = true;
        end
        
        %Planes = zeros(10,2);
%        load icra_path1.mat;
%        path_rrt = flipud(path_rrt);
 %       Planes1 = path_rrt';
        
        

        
        %msgpack('pack',Planes)
    end
    
 %       load test_path_sm.mat;
        load test_path_RP1119_case6.mat;
 %       path_rrt2 = flipud(path_rrt2);
 %       path_rrt = flipud(path_rrt);
        Planes = [path_rrt]';
        
        %Planes(18) = 200;
        zmq('send', p0,  msgpack('pack',Planes));
    
    while udp_recv('getQueueSize', f_camera) > 0
        recv_items = recv_items + 1;
        udp_data = udp_recv('receive',f_camera);
        [metadata, offset] = msgpack('unpack', udp_data);
        msg_id = char(metadata.id);
        % This must be uint8
        raw = udp_data(offset+1:end);
        if strcmp(msg_id,'head_camera')
            count_cam = count_cam + 1;
            data_yuyv.meta = metadata;
            data_yuyv.raw = raw;
            data_yuyv.recv = true;
        end
    end

    while udp_recv('getQueueSize', f_vision) > 0
        recv_items = recv_items + 1;
        udp_data = udp_recv('receive',f_vision);
        [metadata, offset] = msgpack('unpack', udp_data);
        msg_id = char(metadata.id);
        % This must be uint8
        raw = udp_data(offset+1:end);
        if strcmp(msg_id,'detect')
            count_detect = count_detect + 1;
            data_detect.meta = metadata;
            %data_detect.raw = raw;
            data_detect.recv = true;
        end
    end
    
    while udp_recv('getQueueSize', f_label) > 0
        recv_items = recv_items + 1;
        udp_data = udp_recv('receive', f_label);
        [metadata, offset] = msgpack('unpack', udp_data);
        msg_id = char(metadata.id);
        % This must be uint8
        raw = udp_data(offset+1:end);
        if strcmp(msg_id,'labelA')
            count_labelA = count_labelA + 1;
            data_labelA.meta = metadata;
            data_labelA.raw = raw;
            data_labelA.recv = true;
        end
        if strcmp(msg_id,'labelB')
            count_labelB = count_labelB + 1;
            data_labelB.meta = metadata;
            data_labelB.raw = raw;
            data_labelB.recv = true;
        end
    end

    while udp_recv('getQueueSize', f_world) > 0
        recv_items = recv_items + 1;
        udp_data = udp_recv('receive',f_world);
        [metadata, offset] = msgpack('unpack', udp_data);
        msg_id = char(metadata.id);
        % This must be uint8
        raw = udp_data(offset+1:end);
        if strcmp(msg_id,'world')
            count_world = count_world + 1;
            data_world.meta = metadata;
            %data_world.raw = raw;
            data_world.recv = true;
        end
    end

    t=toc;
    t_last = t;
    last_draw_duration = 0;

    if (t>last_draw_time+0.066) && (data_world.recv || data_detect.recv || data_labelA.recv || data_labelB.recv || data_yuyv.recv)

        if data_world.recv, monitor.process_msg(data_world.meta,[],cam); end
        if data_detect.recv, monitor.process_msg(data_detect.meta,[],cam); end
        if data_labelA.recv, monitor.process_msg(data_labelA.meta,data_labelA.raw,cam); end
        if data_labelB.recv, monitor.process_msg(data_labelB.meta,data_labelB.raw,cam); end
        if data_yuyv.recv, monitor.process_msg(data_yuyv.meta,data_yuyv.raw,cam); end
        drawnow;
        t1=toc;
        last_draw_duration = t1-t_last;
        %      disp(sprintf('draw time: %.2f ms / %.2f fps',last_draw_duration*1000,1/(t-last_draw_time)));
        data_world.recv = false;
        data_detect.recv = false;
        data_labelA.recv = false;
        data_labelB.recv = false;
        data_yuyv.recv = false;
        last_draw_time = t1;
    end
end
