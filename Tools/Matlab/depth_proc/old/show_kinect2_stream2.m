clear all;
% close all;
% 4 bytes in float (single precision)
DEPTH_W = 512;
DEPTH_H = 424;
DEPTH_MAX = 2000;%8000;
DEPTH_MIN = 200;
figure(1);
h_depth = imagesc(zeros(DEPTH_H, DEPTH_W));
caxis([DEPTH_MIN DEPTH_MAX]);
%
RGB_W = 1920;
RGB_H = 1080;
rgb_img = uint8(zeros([RGB_H, RGB_W, 3]));
figure(2);
h_rgb = image(rgb_img);

%{
if metadata.id
if DEPTH_W * DEPTH_H * 4 ~= metadata.rsz
    disp('Bad metadata!');
else
    disp('ok');
    %raw = reshape(typecast(raw, 'single'), [DEPTH_W, DEPTH_H]);
    %set(h_depth, 'CData', raw');
end
%}

% 1 second timeout
s_depth = zmq('subscribe', 'tcp', '*', 43346);
s_color = zmq('subscribe', 'tcp', '*', 43347);
s_mesh = zmq('subscribe', 'tcp', '*', 43344);

log = 1;
fig_id = 0;
folder_id = 1;
CLICK = [];
while 1
    idx = zmq('poll',1000);  % assume only one channel
    if isempty(idx)
        disp('empty!');
        return;
    end
    for s = 1:numel(idx)
        s_idx = idx(s);
        [data, has_more] = zmq('receive', s_idx);
        % Get the metadata
        [metadata,offset] = msgpack('unpack', data);
        if has_more, [raw, has_more] = zmq('receive', s_idx); end
        char(metadata.id)
        if strcmp(char(metadata.id), 'k2_depth')
            if DEPTH_W * DEPTH_H * 4 ~= metadata.rsz
                disp('Bad k2_depth metadata!');
            else
                raw = reshape(typecast(raw, 'single'), [DEPTH_W, DEPTH_H]);
                 figure(1), imagesc(raw');      
                 
                 if log == 1
                   if fig_id < 1000 && folder_id < 6
                       fig_id = fig_id + 1;
                       save(strcat('Log_0122_1/',int2str(folder_id),'/depth_',int2str(fig_id),'.mat'),'raw','metadata');
                   elseif fig_id == 1000 && folder_id < 6
                       folder_id = folder_id + 1;                 
                       fig_id = 0;
                   end      

                 else

                % pose 
                T_ = reshape(metadata.tr,4,4)';
                
                Euler = [0; 0.7; 0]*pi/180;
                R_ = eulr2dcm(Euler);
                param{1} = T_(1:3,1:3)*R_'; % orientation 
                param{2} = T_(1:3,4);       % translation
%                 
%                   
%                 CLICK = ginput(1);
%                
%                 if ~isempty(CLICK)
%                  % param{3} = struct('mode','manual_3d','data',[ 0 0 0]);
%                  param{3} = struct('mode','manual_2d','data',CLICK);  
%                 else
                  param{3} = struct('mode',[],'data',[]);
%           
%                 end                
%                 %    if mod(s,3) == 0
                  Planes = detectPlaneInstances_kinect_v2(raw,param,4);  
                 end
                 
                disp('done');
               
            end
        elseif strcmp(char(metadata.id), 'k2_rgb')
            rgb_img = djpeg(raw);
            set(h_rgb, 'CData', rgb_img);
         
        else
            disp(char(metadata.id));
            

         %   if fig_id < 100
         %       fig_id = fig_id + 1;
         %       save(strcat('Temp_figures/LIDARraw_',int2str(fig_id),'.mat'),'raw','metadata');
          %  end  
            
        end
    end
    drawnow;
end