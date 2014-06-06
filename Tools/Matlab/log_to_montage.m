datestamp = '06.06.2014.09.11.52';
% Metadata
fid = fopen(sprintf('Logs/yuyv_m_%s.log',datestamp));
yuyvMeta = fread(fid,Inf,'*uint8');
fclose(fid);
clear fid;
yuyvMeta = msgpack('unpacker',yuyvMeta,'uint8');

% Setup the colortable item
% Grab the YUYV logged information
f_raw = fopen(sprintf('Logs/yuyv_r_%s.log',datestamp));
yuyvMontage = fread(f_raw,Inf,'*uint32');
yuyvMontage = reshape( ...
    yuyvMontage, ...
    [yuyvMeta{1}.w/2, yuyvMeta{1}.h, 1, numel(yuyvMeta)] ...
    );
save(sprintf('Logs/yuyv_%s.mat',datestamp),'yuyvMontage');

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