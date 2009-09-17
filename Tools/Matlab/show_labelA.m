clear all;
% raw labelA
%fid = fopen('Data/labelA.raw');A = fread(fid,Inf,'*uint8');fclose(fid);
%A = reshape(A,[320,240])'; figure(1); imagesc(A);
% zlib labelA
fid = fopen('Data/labelA_z.raw');Az = fread(fid,Inf,'*uint8');fclose(fid);
Az = reshape(zlibUncompress(Az),[320,240])';
figure(2); i_az = image(Az);
cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
cmap=[cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];
colormap(cmap);
% JPEG yuyv
fid = fopen('Data/labelA.jpeg');
jimg = fread(fid,Inf,'*uint8');fclose(fid);
jimg = djpeg(jimg);
figure(3); imagesc(jimg);
drawnow;

%{
% Grab the YUYV logged information
fid = fopen('Data/yuyv_r_09.17.2009.01.48.45.log');
yuyvMontage = fread(fid,Inf,'*uint32');
fclose(fid);
% Metadata
fid = fopen('Data/yuyv_m_09.17.2009.01.48.45.log');
yuyvMeta = fread(fid,Inf,'*uint8');
fclose(fid);
yuyvMeta = msgpack('unpacker',yuyvMeta,'uint8');
% Setup the colortable item
n_img = numel(yuyvMontage) / yuyvMeta{1}.w / yuyvMeta{1}.h * 2;
yuyvMontage = reshape(yuyvMontage,[yuyvMeta{1}.w/2,yuyvMeta{1}.h,1,n_img]);
save('test.mat','yuyvMontage');
%}

% Add the monitor
head_zlib_fd  = udp_recv('new', 54321);
s_head_zlib = zmq( 'fd', head_zlib_fd );
while 1
    % 1 second timeout
    idx = zmq('poll',1000);
    if numel(idx)>0
        [s_data,has_more] = zmq( 'receive', s_head_zlib );
        while udp_recv('getQueueSize',head_zlib_fd) > 0
            udp_data = udp_recv('receive',head_zlib_fd);
        end
        [metadata,offset] = msgpack('unpack',udp_data);
        lA_z = udp_data(offset+1:end); % This must be uint8
        Az = reshape(zlibUncompress(lA_z),[metadata.w,metadata.h])';
        set(i_az,'Cdata', Az);
        drawnow;
    end
end