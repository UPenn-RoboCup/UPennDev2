%% Access to JPEG and msgpack mex files
addpath( genpath('.') );

%% Acquire the metadata
%fid = fopen('~/Dropbox/Domenico/head_camera_10.24.2013.16.41.09_meta.log');
fid = fopen('~/Dropbox/Domenico/lwrist_camera_10.24.2013.16.41.09_meta.log');
msg = fread(fid,inf,'*uchar');
fclose(fid);
objs = msgpack('unpacker', msg);
clear msg;

%% Acquire the jpeg data
%fid = fopen('~/Dropbox/Domenico/head_camera_10.24.2013.16.41.09_raw.log');
fid = fopen('~/Dropbox/Domenico/lwrist_camera_10.24.2013.16.41.09_raw.log');
figure(1);
clf;
for i=1:numel(objs)
    jimg = fread(fid,objs{i}.sz,'*uchar');
    log_img = djpeg(jimg);
    imagesc(log_img);
    drawnow;
end
fclose(fid);

%% Aquire the body joint angles
fid = fopen('~/Dropbox/Domenico/body_10.24.2013.17.46.16_meta.log');
msg = fread(fid,inf,'*uchar');
fclose(fid);
jobjs = msgpack('unpacker', msg);
clear msg;