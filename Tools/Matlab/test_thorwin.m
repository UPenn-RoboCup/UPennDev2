% Clear the environment
clear all;
close all;
startup;

%% Camera Figure
% Colormap for the labeled image
cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
cmap = [cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];
figure(1);
clf;
% (top)
% LabelA
%f_lA = subplot(2,3,2);
f_lA = axes('Units','Normalized','position',[0 0.5 0.3 0.5]);
im_lA = image(zeros(1));
colormap(cmap);
hold on;
p_ball = plot([0],[0], 'y*');
% Remove from the plot
set(p_ball,'Xdata', []);
set(p_ball,'Ydata', []);
r_ball = rectangle('Position', [0 0 1 1],... 
  'Curvature',[1,1], 'EdgeColor', 'b', 'LineWidth', 2);
p_post = cell(2,1);
for i=1:numel(p_post)
    p_post{i} = plot([0],[0], 'b-', 'LineWidth', 2);
    % Remove from the plot
    set(p_post{i},'Xdata', []);
    set(p_post{i},'Ydata', []);
end
% Assume up to 3 obstacles
h_obstacle = cell(3,1);
for i=1:numel(h_obstacle)
    h_obstacle{i} = plot([0],[0], 'r-', 'LineWidth', 2);
    % Remove from the plot
    set(h_obstacle{i},'Xdata', []);
    set(h_obstacle{i},'Ydata', []);
end

% yuyv
%f_yuyv = subplot(2,3,5);
f_yuyv = axes('Units','Normalized','position',[0.3 0.5 0.3 0.5]);
im_yuyv = image(zeros(1));


% Save the camera handles
cam = {};
cam.f_lA = f_lA;
cam.im_lA = im_lA;
cam.f_yuyv = f_yuyv;
cam.im_yuyv = im_yuyv;
cam.p_ball = p_ball;
cam.r_ball = r_ball;
cam.p_post = p_post;
cam.h_obstacle = h_obstacle;
% Plot scale
% Default: labelA is half size, so scale twice
scale = 2;
cam.scale = 2;

%% Localization
%cam.h_field = subplot(2,3,1);
cam.h_field = axes('Units','Normalized','position',[0.3 0 0.3 0.5]);

% Show the field here
plot_field(cam.h_field,2);
% Camera 1 Debug messages
cam.a_debug_ball = annotation('textbox',...
    [0.6 0 0.13 1],...
    'String','Top Camera',...
    'FontSize',10,...
    'FontName','Arial',...
    'LineStyle','--',...
    'EdgeColor',[1 1 0],...
    'LineWidth',2,...
    'BackgroundColor',[0.9  0.9 0.9],...
    'Color',[0.84 0.16 0]);
cam.a_debug_goal = annotation('textbox',...
    [0.73 0 0.13 1],...
    'String','Top Camera',...
    'FontSize',10,...
    'FontName','Arial',...
    'LineStyle','--',...
    'EdgeColor',[1 1 0],...
    'LineWidth',2,...
    'BackgroundColor',[0.9  0.9 0.9],...
    'Color',[0.84 0.16 0]);
cam.a_debug_obstacle = annotation('textbox',...
    [0.86 0 0.14 1],...
    'String','Top Camera',...
    'FontSize',10,...
    'FontName','Arial',...
    'LineStyle','--',...
    'EdgeColor',[1 1 0],...
    'LineWidth',2,...
    'BackgroundColor',[0.9  0.9 0.9],...
    'Color',[0.84 0.16 0]);

% World debug
cam.w_debug = annotation('textbox',...
    [0 0 0.3 0.5],...
    'String','Localization',...
    'FontSize',12,...
    'FontName','Arial'...
);
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
last_draw_duration = 0

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
            end            
        end
    end
    t=toc;
%    disp(sprintf('recv time: %.2f ms %d items',(t-t_last)*1000, recv_items));
    t_last = t;
    last_draw_duration = 0;

    if (t>last_draw_time+0.066) && (data_world.recv || data_detect.recv || data_labelA.recv) 

      if data_world.recv process_libVision_msg(data_world.meta,data_world.raw,cam); end
      if data_detect.recv process_libVision_msg(data_detect.meta,data_detect.raw,cam); end
      if data_labelA.recv process_libVision_msg(data_labelA.meta,data_labelA.raw,cam); end
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
