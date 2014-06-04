clear all;
close all;
startup;
% Camera data for MATLAB GUI
camera = {};   % send everything over a single channel, may cause jam...

%% Camera Image Display
% Colormap for the labeled image
cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
cmap = [cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];
figure(1);
clf;
% yuyv
camera.f_yuyv = subplot(2,3,4);
camera.im_yuyv = image(zeros(1));
title('YUYV');
%set(gca,'Visible','off');

% LabelA / LabelB
camera.f_lA = subplot(2,3,2);
camera.im_lA = image(zeros(1));
% colormap(cmap);
hold on;
labelA.p_ball = plot([0],[0],'m*');
hold off;
title('label A');

% LabelB
camera.f_lB = subplot(2,3,5);
camera.im_lB = image(zeros(1));
% colormap(cmap);
title('labelB');

%% Localization Display
h_field = subplot(2,3,1);
% Show the field here
plot_field(h_field,2)
% Debug messages
camera.a_debug = annotation('textbox',...
    [0.65 0 0.35 1],...
    'String','Debug Message',...
    'FontSize',14,...
    'FontName','Arial',...
    'LineStyle','--',...
    'EdgeColor',[1 1 0],...
    'LineWidth',2,...
    'BackgroundColor',[0.9  0.9 0.9],...
    'Color',[0.84 0.16 0]);

% % World debug
% w_debug = annotation('textbox',...
%     [0 0 0.5 0.5],...
%     'String','Localization',...
%     'FontSize',14,...
%     'FontName','Arial'...
% );
    
%% Network setup
% Add the monitor
REAL_ROBOT = 0;
% REAL_ROBOT = 1;
if REAL_ROBOT==1 
    camera.fd = udp_recv('new', 33333);
    s_top = zmq( 'fd', camera.fd );
else
    camera.s = zmq( 'subscribe', 'ipc', 'top' );
end

% Plot scale
% Default: labelA is half size, so scale twice
camera.scale = 2;

%% Main loop
drawnow;
needs_draw = 0;
global logging LOGGER;
logging = false;
%logging = true;
LOGGER = thor_logger();
LOGGER.init();
while 1
    % 1 second timeout
    idx = zmq('poll',1000);  % assume only one channel
    for s = 1:numel(idx)
        if isfield(camera, 'fd')
          while udp_recv('getQueueSize',camera.fd) > 0
              data = udp_recv('receive',camera.fd);
              % Get the metadata
              [metadata,offset] = msgpack('unpack',data);

              fprintf(metadata.meta_yuyv.id);

              % Process the raw data
              raw = data(offset+1:end); % This must be uint8
              [draw] = thor_process_monitor_message(metadata, raw, camera);
              if draw; needs_draw = 1; end;
          end  
        else
          while 1
            %zmq
            s_idx = idx(s);
            [data, has_more] = zmq( 'receive', s_idx );
            % Get the metadata
            [metadata,offset] = msgpack('unpack',data);
            % Process the raw data
            raw = data(offset+1:end); % This must be uint8
            [has_draw] = thor_process_monitor_message(metadata, raw, camera);
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
