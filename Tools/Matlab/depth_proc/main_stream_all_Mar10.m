clear all;
close all;

% 4 bytes in float (single precision)
DEPTH_W = 512;
DEPTH_H = 424;
DEPTH_MAX = 2000;%8000;
DEPTH_MIN = 200;

datetime.setDefaultFormats('defaultdate','yyyy-MM-dd');

figure(1);
h_depth = imagesc(zeros(DEPTH_H, DEPTH_W));
caxis([DEPTH_MIN DEPTH_MAX]);
%
RGB_W = 1920;
RGB_H = 1080;
rgb_img = uint8(zeros([RGB_H, RGB_W, 3]));
figure(2);
h_rgb = image(rgb_img);

% 1 second timeout
s_depth = zmq('subscribe', 'tcp', '192.168.123.222', 43346);
s_color = zmq('subscribe', 'tcp', '192.168.123.222', 43347);
s_mesh = zmq('subscribe', 'tcp', '192.168.123.222', 43344);

log = 1;
fig_id = 0;
folder_id = 1;
CLICK = [];
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
           
            if DEPTH_W * DEPTH_H * 4 ~= metadata.rsz
                disp('Bad k2_depth metadata!');
            else
                
                raw = reshape(typecast(raw, 'single'), [DEPTH_W, DEPTH_H]);
               % uisetting; % See uisetting.m                
               % res = depth_proc(raw, metadata, ui);
                figure(1), imagesc(raw');            
            end
        elseif strcmp(char(metadata.id), 'k2_rgb') %%%%%%%%%%%%%%%%%%%%%%%%% RGB
            rgb_img = djpeg(raw);
            set(h_rgb, 'CData', rgb_img);            
         
        else
            disp(char(metadata.id));
         
        end
    end
    drawnow;
end