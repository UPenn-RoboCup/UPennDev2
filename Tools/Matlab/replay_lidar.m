%theta = 0:0.25:270;
%theta = theta*pi/180;

%% Grab list
datestamp0 = '12.31.2008.19.17.58';
prefix = 'lidar_m_';
nprefix = numel(prefix);
log_files0 = dir(strcat('Data/',prefix,'*.log'));
n0 = numel(log_files0);
for idx0=1:n0
    if strfind(log_files0(idx0).name, datestamp0)
        break;
    end
end
fprintf('Timestamp0: %s (Index0 %d of %d)\n', datestamp0, idx0, n0);
log_files = log_files0(idx0:end);

lidar = {};

n = numel(log_files);
for idx=1:n
    
    datestamp = log_files(idx).name(nprefix+1:end-4);
    fprintf('Timestamp: %s (Index %d of %d)\n', datestamp, idx, n);

    fid = fopen(strcat('Data/lidar_m_',datestamp,'.log'));
    msg = fread(fid,inf,'*uchar');
    fclose(fid);
    clear fid;
    lobjs = msgpack('unpacker', msg);
    clear msg;

f_raw = fopen(sprintf('Data/lidar_r_%s.log', datestamp));

for i=1:numel(lobjs)
    meta = lobjs{i};
    raw_lidar = fread(f_raw, lobjs{i}.rsz, '*uchar');
    real_lidar = typecast(raw_lidar, 'single')';
    metadata = rmfield(meta, {'angle', 'rsz'});
    metadata.scan = real_lidar;

    lidar{numel(lidar)+1} = metadata;
    
%    polar(theta, real_lidar);
%    pause(0.025);
end

fclose(f_raw);

end

save('lidar.mat', 'lidar');