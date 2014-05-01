clear all;
%h = 120;
%w = 160;
h = 116;
w = 156;

% raw edges
fid = fopen('Data/edge.raw');A = fread(fid,Inf,'*int32');fclose(fid);
A = double(reshape(A,[w, h])');
% char edges
fid = fopen('Data/edge_char.raw');B = fread(fid,Inf,'*int8');fclose(fid);
B = double(reshape(B,[w, h])');

figure(1);
imshow('Data/edge_img.jpeg');
figure(2);
imagesc(A);
figure(3);
imagesc(B);
figure(4);
hist(A);
figure(5);
imshow('Data/edge_gray.jpeg');