clear all;
close all;

% 4 bytes in float (single precision)
DEPTH_W = 512;
DEPTH_H = 424;

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
       % disp('empty!');
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
            ui.taskMode = 11 ;
            ui.figures(3) = 0;
                    
            [Planes, meta] = detectPlanes6(raw, metadata, ui);  
            [distance, yaw, id] = localizeDoor_v1L(Planes);
            if numel(distance)> 0
                distance
                yaw

                data = struct('dist',distance, 'yaw',yaw); % yaw degree
                packed_data=msgpack('pack',data);
                % distance
                % yaw
                zmq('send',s_field,packed_data);
            end   
             
        end
    end
    drawnow;
end