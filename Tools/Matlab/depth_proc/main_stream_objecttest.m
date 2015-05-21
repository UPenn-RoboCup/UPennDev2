clear all;
close all;

% 4 bytes in float (single precision)
DEPTH_W = 512;
DEPTH_H = 424;
DEPTH_MAX = 2000;%8000;
DEPTH_MIN = 200;

datetime.setDefaultFormats('defaultdate','yyyy-MM-dd');
%
RGB_W = 1920;
RGB_H = 1080;
rgb_img = uint8(zeros([RGB_H, RGB_W, 3]));

% 1 second timeout
s_depth = zmq('subscribe', 'tcp', '192.168.123.246', 43346);
s_color = zmq('subscribe', 'tcp', '192.168.123.246', 43347);
%s_mesh = zmq('subscribe', 'tcp', '192.168.123.246', 43344);
%s_mesh = zmq( 'subscribe', 'ipc', 'mesh0' );

count = 0;
while 1
    idx = zmq('poll',1000);  % assume only one channel
    if isempty(idx)
        disp('empty!');
       % return;
    end
    for s = 1:numel(idx)
        s_idx = idx(s);
        [data, has_more] = zmq('receive', s_idx);
        % Get the metadata
        [metadata,offset] = msgpack('unpack', data);
        if has_more, [raw, has_more] = zmq('receive', s_idx); end
        char(metadata.id)
        if strcmp(char(metadata.id), 'k2_depth') %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% depth 
                raw = reshape(typecast(raw, 'single'), [DEPTH_W, DEPTH_H]);
                uisetting; % See uisetting.m       size(D)
                 
                % res = depth_proc(raw, metadata, ui);
                 
%                 % TASK: localization at the corner 
%                 ui.taskMode = 11 ;
%                 ui.figures(3) = 2;
%                 % average 
%                 [res, meta] = detectPlanes6(raw, metadata, ui);                  
%                 res{1}
%                 pose = localizeCorner_v4(res,metadata)
%                 % TASK: localization at the corner 
                
                 % TASK: localization at the corner 
                ui.taskMode = 11 ;
                ui.figures(3) = 2;
                % average 
                [res, meta, outOfWall] = removeBackgroundPlane(raw, metadata, ui);   
                %pose = localizeCorner_v4(res,metadata)
                % TASK: localization at the corner 
                % raw(outOfWall);               
                
                figure(1), imagesc(raw');            
             % end
        elseif strcmp(char(metadata.id), 'k2_rgb') %%%%%%%%%%%%%%%%%%%%%%%%% RGB
            % rgb_img = djpeg(raw);
            % set(h_rgb, 'CData', rgb_img);   
        end
    end
    drawnow;
end