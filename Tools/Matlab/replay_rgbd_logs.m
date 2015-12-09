%% Read the D
logdir = 'Data/';
prefix = 'k_depth';
suffix = '.log';
d = dir(strcat('Data/', prefix, '_m_*', suffix));

w=320;
h=240;

h_f = figure(1);
h_a = gca;
h_img = imagesc;

for i=1:numel(d)
    fname = d(i).name;
    fid = fopen(fname);
    msg = fread(fid, inf, '*uchar');
    fclose(fid);
    objs = msgpack('unpacker', msg);
    clear msg fid;
    timestamp = fname(numel(prefix)+4:end-numel(suffix));
    fname = strcat(logdir, prefix, '_r_', timestamp, suffix);
    fid = fopen(fname);
    for o=1:numel(objs)
        metadata = objs{o};
        img = fread(fid, w*h, '*uint16');
        img = reshape(img, [w, h])';
        set(h_img,'cdata', img);
        title(h_a,sprintf('Log %d | %d',i,o));
        %log_img = djpeg(jimg);
        pause(1/30);
    end
    fclose(fid);
end

%% Read the RGB
logdir = 'Data/';
prefix = 'k_rgb';
suffix = '.log';
d = dir(strcat('Data/', prefix, '_m_*', suffix));

w=320;
h=240;

h_f = figure(2);
h_a = gca;
h_img = imagesc;

for i=1:numel(d)
    fname = d(i).name;
    fid = fopen(fname);
    msg = fread(fid, inf, '*uchar');
    fclose(fid);
    objs = msgpack('unpacker', msg);
    clear msg fid;
    timestamp = fname(numel(prefix)+4:end-numel(suffix));
    fname = strcat(logdir, prefix, '_r_', timestamp, suffix);
    fid = fopen(fname);
    for o=1:numel(objs)
        metadata = objs{o};
        img = fread(fid, w*h*3, '*uint8');
        img = reshape(img, [3, w, h]);
        img = permute(img, [3,2,1]);
        img = flipud(img);
        set(h_img,'cdata', img);
        title(h_a,sprintf('Log %d | %d',i,o));
        %log_img = djpeg(jimg);
        pause(1/30);
    end
    fclose(fid);
end
