clear all;
% raw edges
fid = fopen('Data/edge.raw');A = fread(fid,Inf,'*int32');fclose(fid);
A = double(reshape(A,[156, 116])');
% char edges
fid = fopen('Data/edge_char.raw');B = fread(fid,Inf,'*int8');fclose(fid);
B = double(reshape(B,[156, 116])');

figure(1);
imagesc(A);

figure(2);
imagesc(B);