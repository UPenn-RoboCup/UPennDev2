fid = fopen('Data/labelA.raw');A = fread(fid);fclose(fid);
A = reshape(A,[320,240])'; figure(1); imagesc(A);
%
fid = fopen('Data/labelA_z.raw');Az = fread(fid,Inf,'uint8');fclose(fid);
Az = reshape(zlibUncompress(cast(Az,'uint8')),[320,240])';
figure(2);
i_az = imagesc(Az);
% JPEG

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