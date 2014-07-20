datestamp = '07.20.2014.08.36.39'
% Metadata
%fid = fopen(sprintf('logs/brazil_robot/yuyv_m_%s.log',datestamp));
fid = fopen(sprintf('logs/brazil_cam_only/yuyv_m_%s.log',datestamp));
yuyvMeta = fread(fid,Inf,'*uint8');
fclose(fid);
clear fid;
yuyvMeta = msgpack('unpacker',yuyvMeta,'uint8');

% Setup the colortable item
% Grab the YUYV logged information
%f_raw = fopen(sprintf('logs/brazil_robot/yuyv_r_%s.log',datestamp));
f_raw = fopen(sprintf('logs/brazil_cam_only/yuyv_r_%s.log',datestamp));
yuyvMontage = fread(f_raw,Inf,'*uint32');
%disp(yuyvMeta{1}.w)
%disp( yuyvMeta{1}.h)
%disp( numel(yuyvMeta))
yuyvMontage = reshape( ...
    yuyvMontage, ...
    [yuyvMeta{1}.w/2, yuyvMeta{1}.h, 1, numel(yuyvMeta)] ...
    );
save(sprintf('logs/yuyv_%s.mat',datestamp),'yuyvMontage');

% Individual reading
%{
f_raw = fopen(sprintf('Data/yuyv_r_%s.log',datestamp));
for i=1:numel(yuyvMeta)
    meta = yuyvMeta{i};
    rsz = meta.w * meta.h * 2;
    if rsz ~= meta.rsz
        disp('Bad metadata!');
    end
    yuyv = fread(f_raw, rsz, '*uint8');
    yuyv = typecast(yuyv,'uint32');
    yuyv = reshape( yuyv, meta.w/2, meta.h );
    yuv = yuyv2yuv(yuyv);
    rgb = ycbcr2rgb(yuv);
    imagesc(rgb);
    pause;
end
clear yuyv yuv rgb;
fclose(f_raw);
clear f_raw;
%}
