%% Clear the environment
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
f_lA = subplot(2,1,1);
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
f_yuyv = subplot(2,1,2);
im_yuyv = image(zeros(1));
hold off;

drawnow;

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
figure(2);
clf;
cam.h_field = subplot(2,2,1);
% Show the field here
plot_field(cam.h_field,2);
% Camera 1 Debug messages
cam.a_debug = annotation('textbox',...
    [0.5 0.5 0.5 0.5],...
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
    [0 0 0.5 0.5],...
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
do_draw = 1;
while running
    % Drawing
    if do_draw==1
        do_draw = 0;
        drawnow;
    end
    % 1 second timeout
    idx = zmq('poll', 1000);
    for s=1:numel(idx)
        s_idx = idx(s);
        s_idx_m = s_idx + 1;
        [s_data, has_more] = zmq( 'receive', s_idx );
        while udp_recv('getQueueSize', fd) > 0
            udp_data = udp_recv('receive',fd);
            [metadata, offset] = msgpack('unpack', udp_data);
            % This must be uint8
            raw = udp_data(offset+1:end);
            is_draw = process_libVision_msg(metadata, raw, cam);
            do_draw = do_draw || is_draw;
        end
    end
end
