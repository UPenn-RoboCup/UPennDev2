clear all;
close all;
startup;
% Camera data for MATLAB GUI
cams = {};
cams{1} = {};
cams{2} = {};

%% Camera Image Display
% Colormap for the labeled image
cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
cmap = [cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];
figure(1);
clf;
% (top)
% yuyv
cams{1}.f_yuyv = subplot(2,3,1);
cams{1}.im_yuyv = image(zeros(1));
hold on;
cams{1}.p_ball = plot([0],[0],'m*');
hold off;
title('Top YUYV');
%set(gca,'Visible','off');
% LabelA
cams{1}.f_lA = subplot(2,3,2);
cams{1}.im_lA = image(zeros(1));
colormap(cmap);
title('Top labelA');
% LabelB
cams{1}.f_lB = subplot(2,3,3);
cams{1}.im_lB = image(zeros(1));
colormap(cmap);
title('Top labelB');
% (bottom)
% yuyv
cams{2}.f_yuyv = subplot(2,3,4);
cams{2}.im_yuyv = image(zeros(1));
hold on;
cams{2}.p_ball = plot([0],[0],'m*');
hold off;
title('Bottom YUYV');
% LabelA 
cams{2}.f_lA = subplot(2,3,5);
cams{2}.im_lA = image(zeros(1));
colormap(cmap);
title('Bottom labelA');
% LabelB
cams{2}.f_lB = subplot(2,3,6);
cams{2}.im_lB = image(zeros(1));
colormap(cmap);
title('Bottom labelB');

%% Localization Display

figure(2);
clf;
h_field = subplot(2,2,1);
% Show the field here
plot_field(h_field,2)
% Camera 1 Debug messages
cams{1}.a_debug = annotation('textbox',...
    [0.5 0.5 0.5 0.5],...
    'String','Top Camera',...
    'FontSize',14,...
    'FontName','Arial',...
    'LineStyle','--',...
    'EdgeColor',[1 1 0],...
    'LineWidth',2,...
    'BackgroundColor',[0.9  0.9 0.9],...
    'Color',[0.84 0.16 0]);
% Camera 2 debug
cams{2}.a_debug = annotation('textbox',...
    [0.5 0 0.5 0.5],...
    'String','Bottom Camera',...
    'FontSize',14,...
    'FontName','Arial',...
    'LineStyle','--',...
    'EdgeColor',[1 1 0],...
    'LineWidth',2,...
    'BackgroundColor',[0.9  0.9 0.9],...
    'Color',[0.84 0.16 0]);
% World debug
w_debug = annotation('textbox',...
    [0 0 0.5 0.5],...
    'String','Localization',...
    'FontSize',14,...
    'FontName','Arial'...
);
    %'LineStyle','--',...
    %'EdgeColor',[1 1 0],...
    %'LineWidth',2,...
    %'BackgroundColor',[0.9  0.9 0.9],...
    %'Color',[0.84 0.16 0]
    
%% Network setup
% Add the monitor
REAL_ROBOT = 0;
if REAL_ROBOT==1 
    cams{1}.fd = udp_recv('new', 33333);
    s_top = zmq( 'fd', cams{1}.fd );
    cams{2}.fd = udp_recv('new', 33334);
    s_bottom = zmq( 'fd', cams{2}.fd );
else
    cams{1}.s = zmq( 'subscribe', 'ipc', 'top' );
    cams{2}.s = zmq( 'subscribe', 'ipc', 'bottom' );
end

% Plot scale
% Default: labelA is half size, so scale twice
cams{1}.scale = 2;
cams{2}.scale = 2;

%% Main loop
drawnow;
needs_draw = 0;
while 1
    % 1 second timeout
    idx = zmq('poll',1000);
    for s=1:numel(idx)
        s_idx = idx(s);
        s_idx_m = s_idx + 1;
        cam = cams{s_idx_m};
        % Receive the data (TODO: Non-blocking or has_more)
        if isfield(cam, 'fd')
            while udp_recv('getQueueSize',cam.fd) > 0
                data = udp_recv('receive',cam.fd);
                % Get the metadata
                [metadata,offset] = msgpack('unpack',data);
                % Process the raw data
                raw = data(offset+1:end); % This must be uint8
                process_monitor_message(metadata, raw, cam);
            end
        else
            while 1
                % Receive
                [data, has_more] = zmq( 'receive', s_idx );
                % Get the metadata
                [metadata,offset] = msgpack('unpack',data);
                % Process the raw data
                raw = data(offset+1:end); % This must be uint8
                [has_draw] = process_monitor_message(metadata, raw, cam);
                if has_draw; needs_draw = 1; end;
                % Get more data
                if has_more<1; break; end;
            end
        end 
    end
    % Draw after processing everything
    if needs_draw>0
        drawnow;
        needs_draw = 0;
    end
    
end