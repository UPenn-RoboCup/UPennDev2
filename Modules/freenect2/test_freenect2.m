w = 512;
h = 424;

% Show the raw IR image
fid = fopen('Data/ir.raw');
raw = fread(fid,Inf,'*single');
fclose(fid);
ir_raw = reshape(raw, [w, h])';
clear raw;

% Show the depth image
fid = fopen('Data/depth.raw');
raw = fread(fid,Inf,'*single');
fclose(fid);
depth_raw = reshape(raw, [w, h])';
clear raw;

% Show the rgb image
fid = fopen('Data/rgb.raw');
raw = fread(fid,Inf,'*uint8');
fclose(fid);
rgb_raw = reshape(raw, [3, 1920*1080]);
rgb = permute(reshape(rgb_raw, [3, 1920, 1080]), [3,2,1]);
clear raw;

figure(1);
clf;
imagesc(ir_raw)
title('Raw IR');

figure(2);
clf;
imagesc(depth_raw)
title('Raw Depth');

figure(3);
clf;
imagesc(rgb)
title('Decompressed RGB');