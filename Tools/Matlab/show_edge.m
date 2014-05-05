%% Cleanup
close all;
clear all;

%% Raw image and sample gray
figure;
subplot(1,2,1);
imshow('Data/edge_img.jpeg');
axis image;
subplot(1,2,2);
imshow('Data/edge_gray.jpeg');
axis image;

%% Edges from LoG convolution
%h = 120;
%w = 160;
h = 116;
w = 156;
% raw edges
fid = fopen('Data/edge.raw');A = fread(fid,Inf,'*int32');fclose(fid);
edge_raw = double(reshape(A,[w, h])');
% char edges
fid = fopen('Data/edge_char.raw');A = fread(fid,Inf,'*int8');fclose(fid);
edge_char = double(reshape(A,[w, h])');
figure;
subplot(1,2,1);
imagesc(edge_raw);
axis image;
subplot(1,2,2);
imagesc(edge_char);
axis image;

%% Line counts and sums
fid = fopen('Data/line_cnts.raw');A = fread(fid,Inf,'*int64');fclose(fid);
line_counts = double(reshape(A,[200, 45])');
% lines sums
fid = fopen('Data/line_sums.raw');A = fread(fid,Inf,'*int64');fclose(fid);
line_sums = double(reshape(A,[200, 45])');
%
figure;
subplot(2,1,1);
imagesc(line_counts);
title('Line Counts');
subplot(2,1,2);
imagesc(line_sums);
title('Line Sums');

%% Line extrema
fid = fopen('Data/line_min.raw');A = fread(fid,Inf,'*int64');fclose(fid);
line_min = double(reshape(A,[200, 45])');
% lines max
fid = fopen('Data/line_max.raw');A = fread(fid,Inf,'*int64');fclose(fid);
line_max = double(reshape(A,[200, 45])');
%
figure;
subplot(2,1,1);
imagesc(line_min);
subplot(2,1,2);
imagesc(line_max);

%% bbox of the image
w = 31; h = 101;
%w = 31; h = 41;
fid = fopen('Data/ys_bbox.raw');A = fread(fid,Inf,'*uint8');fclose(fid);
ys_bbox = double(reshape(A,[w, h])');
fid = fopen('Data/us_bbox.raw');A = fread(fid,Inf,'*uint8');fclose(fid);
us_bbox = double(reshape(A,[w, h])');
fid = fopen('Data/vs_bbox.raw');A = fread(fid,Inf,'*uint8');fclose(fid);
vs_bbox = double(reshape(A,[w, h])');
fid = fopen('Data/trans_bbox.raw');A = fread(fid,Inf,'*double');fclose(fid);
trans_bbox = double(reshape(A,[w, h])');

%
figure;
subplot(2,2,1);
imagesc(ys_bbox);
title('Y');
axis image;
colorbar;
subplot(2,2,2);
imagesc(us_bbox);
title('U');
axis image;
colorbar;
subplot(2,2,3);
imagesc(vs_bbox);
title('V');
axis image;
colorbar;
subplot(2,2,4);
imagesc(trans_bbox);
title('Transformed');
axis image;
colorbar;