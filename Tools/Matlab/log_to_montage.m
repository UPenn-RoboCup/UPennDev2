datestamp = '09.17.2009.01.48.45';
% Metadata
fid = fopen(sprintf('Data/yuyv_m_%s.log',datestamp));
yuyvMeta = fread(fid,Inf,'*uint8');
fclose(fid);
clear fid;
yuyvMeta = msgpack('unpacker',yuyvMeta,'uint8');

% Grab the YUYV logged information
f_raw = fopen(sprintf('Data/yuyv_r_%s.log',datestamp));

for i=1:numel(yuyvMeta)
%for i=1:1
    meta = yuyvMeta{i};
    rsz = meta.w * meta.h * 2;
    if rsz ~= meta.rsz
        disp('Bad metadata!');
    end
    yuyv = fread(f_raw, rsz, '*uint8');
    yuyv = typecast(yuyv,'uint32');
    yuyv = reshape( yuyv, meta.w/2, meta.h );
    s = sum(yuyv(:));
    if s~=0
        yuv = yuyv2yuv(yuyv);
        imagesc(yuv);
        pause;
    end
end

fclose(f_raw);
clear f_raw;

% Setup the colortable item
%n_img = numel(yuyvMontage) / yuyvMeta{1}.w / yuyvMeta{1}.h * 2;
%yuyvMontage = reshape(yuyvMontage,[yuyvMeta{1}.w/2,yuyvMeta{1}.h,1,n_img]);
%save('test.mat','yuyvMontage');