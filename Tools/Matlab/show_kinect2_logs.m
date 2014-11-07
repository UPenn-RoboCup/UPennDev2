% Starting timestamp
%datestamp0 = '11.07.2014.10.22.29';
datestamp0 = '11.07.2014.10.25.37';

prefix = 'k2_depth_m_';
nprefix = numel(prefix);
log_files0 = dir(strcat('Data/',prefix,'*.log'));
n0 = numel(log_files0);
for idx0=1:n0
    if strfind(log_files0(idx0).name, datestamp0)
        break;
    end
end
fprintf('Timestamp0: %s (Index0 %d of %d)\n', datestamp0, idx0, n0);
log_files = log_files0(idx0:end);

% 4 bytes in float (single precision)
DEPTH_W = 512;
DEPTH_H = 424;
DEPTH_MAX = 2000;%8000;
DEPTH_MIN = 200;
%
RGB_W = 1920;
RGB_H = 1080;
rgb_img = uint8(zeros([RGB_H, RGB_W, 3]));

n = numel(log_files);
for idx=1:n
    datestamp = log_files(idx).name(nprefix+1:end-4);
    fprintf('Timestamp: %s (Index %d of %d)\n', datestamp, idx, n);
    % Metadata
    fid = fopen(sprintf('Data/k2_depth_m_%s.log',datestamp));
    depthMeta = fread(fid,Inf,'*uint8');
    fclose(fid);
    depthMeta = msgpack('unpacker', depthMeta, 'uint8');
    % Metadata
    fid = fopen(sprintf('Data/k2_rgb_m_%s.log',datestamp));
    rgbMeta = fread(fid,Inf,'*uint8');
    fclose(fid);
    rgbMeta = msgpack('unpacker',rgbMeta,'uint8');
    
    % Grab the Depth logged information
    f_raw = fopen(sprintf('Data/k2_depth_r_%s.log',datestamp));
    f_rgb = fopen(sprintf('Data/k2_rgb_r_%s.log',datestamp));
    
    % Individual reading
    nlog = numel(depthMeta);
    for ilog=1:nlog
        meta = depthMeta{ilog};
        fprintf('Log %d of %d\n', ilog, nlog);
        if DEPTH_W * DEPTH_H * 4 ~= meta.rsz, disp('Bad metadata!'); end
        depthRaw = fread(f_raw, [DEPTH_W, DEPTH_H], '*single');
        meta = rgbMeta{ilog};
        rgbJPEG = fread(f_rgb, meta.rsz, '*uint8');
        %{
    rgbRaw = fread(f_rgb, [3 RGB_W * RGB_H], '*uint8');
    rgbRaw = permute(reshape(rgbRaw, [3, RGB_W, RGB_H]), [3,2,1]);
    % Color channels are BGR, not RGB...
    rgb_img(:,:,1) = rgbRaw(:,:,3);
    rgb_img(:,:,2) = rgbRaw(:,:,2);
    rgb_img(:,:,3) = rgbRaw(:,:,1);
        %}
        figure(1);
        imagesc(depthRaw');
        caxis([DEPTH_MIN DEPTH_MAX]);
        figure(2);
        rgb_img0 = djpeg(rgbJPEG);
        rgb_img(:,:,1) = rgb_img0(:,:,3);
        rgb_img(:,:,2) = rgb_img0(:,:,2);
        rgb_img(:,:,3) = rgb_img0(:,:,1);
        image(rgb_img);
        pause;
    end
    fclose(f_raw);
    fclose(f_rgb);
end

clear fid f_raw f_rgb;