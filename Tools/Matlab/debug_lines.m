% Grab the common metadata
fid = fopen('Data/lines1/count_d_m.log');
meta = fread(fid, Inf,'*int8');
metadata = msgpack('unpack', meta);
fclose(fid);

% counts
fid = fopen('Data/lines1/count_d_r.log');
cnt = fread(fid,Inf,'*int');
cnt = reshape(cnt, [metadata.maxr, metadata.nth]);
fclose(fid);
figure(1);
imagesc(cnt);

% Show the labelB image
fid = fopen('Data/lines1/labelB_d_r.log');
labelB = fread(fid,Inf,'*uint8');
labelB = reshape(labelB, [metadata.w, metadata.h]);
fclose(fid);

cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
cmap = [cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];
figure(2);
image(labelB');
colormap(cmap);
