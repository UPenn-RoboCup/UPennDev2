clear all;
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

% line counts
fid = fopen('Data/line_cnts.raw');A = fread(fid,Inf,'*int32');fclose(fid);
size(A)
line_counts = double(reshape(A,[180, 45])');
% lines sums
fid = fopen('Data/line_sums.raw');A = fread(fid,Inf,'*int32');fclose(fid);
line_sums = double(reshape(A,[180, 45])');
% lines min
fid = fopen('Data/line_min.raw');A = fread(fid,Inf,'*int32');fclose(fid);
line_min = double(reshape(A,[180, 45])');
% lines max
fid = fopen('Data/line_max.raw');A = fread(fid,Inf,'*int32');fclose(fid);
line_max = double(reshape(A,[180, 45])');

figure(1);
imshow('Data/edge_img.jpeg');
figure(2);
imshow('Data/edge_gray.jpeg');
figure(3);
imagesc(edge_raw);
figure(4);
imagesc(edge_char);
%
figure(5);
imagesc(line_counts);
figure(6);
imagesc(line_sums);
%
figure(7);
imagesc(line_min);
figure(8);
imagesc(line_max);