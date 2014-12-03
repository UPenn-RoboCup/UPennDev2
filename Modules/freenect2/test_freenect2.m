w = 512;
h = 424;

% Show the raw IR image
fid = fopen('ir.raw');
raw = fread(fid,Inf,'*single');
fclose(fid);
ir_raw = reshape(raw, [w, h])';
clear raw;

% Show the depth image
fid = fopen('depth.raw');
raw = fread(fid,Inf,'*single');
fclose(fid);
depth_raw = reshape(raw, [w, h])';
clear raw;

% Show the rgb image
fid = fopen('rgb.raw');
raw = fread(fid,Inf,'*uint8');
fclose(fid);
rgb_raw = reshape(raw, [3, 1920*1080]);
clear raw;
rgb0 = permute(reshape(rgb_raw, [3, 1920, 1080]), [3,2,1]);
% Smarter way to do this?
rgb = uint8(zeros(size(rgb0)));
rgb(:,:,1) = rgb0(:,:,3);
rgb(:,:,2) = rgb0(:,:,2);
rgb(:,:,3) = rgb0(:,:,1);
clear rgb0;

figure(1);
clf;
imagesc(ir_raw)
title('Raw IR');

figure(2);
clf;
imagesc(depth_raw)
title('Raw Depth (centimeters)');

figure(3);
clf;
imagesc(rgb)
title('Decompressed RGB');