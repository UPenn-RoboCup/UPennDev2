clear all;
close all;

% 4 bytes in float (single precision)
DEPTH_W = 512;
DEPTH_H = 424;
DEPTH_MAX = 2000;%8000;
DEPTH_MIN = 200;

run('../startup.m');

% 1 second timeout
s_depth = zmq('subscribe', 'tcp', '192.168.123.246', 43346);
s_color = zmq('subscribe', 'tcp', '192.168.123.246', 43347);
%s_mesh = zmq('subscribe', 'tcp', '192.168.123.232', 43344);
%s_mesh = zmq( 'subscribe', 'ipc', 'mesh0' );
s_field = zmq('publish', 'tcp', 1999);

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
        %char(metadata.id)
        if strcmp(char(metadata.id), 'k2_depth') %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% depth 
                raw = reshape(typecast(raw, 'single'), [DEPTH_W, DEPTH_H]);
                uisetting; % See uisetting.m       size(D)
               
                    % TASK: rough terrain
                    ui.taskMode = 4;
                    ui.figures(3) = 2;
                    % average 
                    [res, meta] = detectPlanes7(raw, metadata, ui);  
                    res
%                     if numel(distance)> 0
%                         %distance
%                         %yaw
%                         
%                         data = struct('dist',distance, 'yaw',yaw); % yaw degree
%                         packed_data=msgpack('pack',data);
%                         % distance
%                         % yaw
%                         zmq('send',s_field,packed_data);
%                      end
              
               % figure(1), imagesc(raw');            
             % end
        elseif strcmp(char(metadata.id), 'k2_rgb') %%%%%%%%%%%%%%%%%%%%%%%%% RGB
            % rgb_img = djpeg(raw);
            % set(h_rgb, 'CData', rgb_img);            
         
        elseif 0 %strcmp(char(metadata.id), 'mesh0')
          
            if (mod(count,3) == 0) 
                
                metadata.dims = metadata.dim;
                metadata.flag = 1;
                raw = reshape(typecast(raw, 'single'), [metadata.dim(2), metadata.dim(1)]);

    %             figure(3), imagesc(raw);
    %              disp(metadata)
    %              size(raw)

               [ Planes ] = detectPlaneInstances_lidar_v5c( raw', 3, metadata);  
            end
           count = count + 1;
        end
    end
    drawnow;
end