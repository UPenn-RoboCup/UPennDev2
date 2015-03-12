% Starting timestamp
prefix0 = 'mesh';
datestamp0 = '03.11.2015.15.24.14';
prefix = strcat(prefix0, '_m_');

log_files0 = dir(strcat('./Data/',prefix,'*.log'));
n0 = numel(log_files0);
for idx0=1:n0
    if strfind(log_files0(idx0).name, datestamp0)
        break;
    end
end
log_files = log_files0(idx0:end);
n = numel(log_files);
clear n0 log_files0

fprintf('Unlog %d %s logs\n', n, prefix0);
for idx=1:n
    datestamp = log_files(idx).name(numel(prefix0)+4:end-4);
    fprintf('Timestamp: %s (Index %d of %d)\n', datestamp, idx, n);
    fid = fopen(strcat('./Data/',prefix0,'_m_',datestamp,'.log'));
    msg = fread(fid,inf,'*uchar');
    fclose(fid);
    clear fid;
    obj = msgpack('unpacker', msg);
    %{
    [obj offset] = msgpack('unpack', msg);
    msg = msg(offset+1:end);
    [obj offset] = msgpack('unpack', msg);
    obj
    offset
    %}
    clear msg;
    fprintf('Unpack %d\n', numel(obj));
    f_raw = fopen(sprintf('./Data/%s_r_%s.log', prefix0, datestamp));
    for i=1:numel(obj)
        raw = fread(f_raw, obj{i}.rsz, '*uchar');
    end
    clear raw_lidar real_lidar;
    fclose(f_raw);
end
