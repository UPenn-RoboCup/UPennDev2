function [] = log_to_montage(logname)

    %logname = 'webots';

    log_dir = strcat('Data/',logname);

    log_list = get_log_list2(log_dir, 'yuyv_m_');

    prefix = strcat(log_dir, '/yuyv_m_');

    for i=1:numel(log_list)

        filename = log_list(i).name;
        datestamp = filename(numel(prefix)+1:end-4);

        % Metadata
        fid = fopen(filename);
        yuyvMeta = fread(fid,Inf,'*uint8');
        fclose(fid);
        clear fid;
        yuyvMeta = msgpack('unpacker',yuyvMeta,'uint8');

        % Setup the colortable item
        % Grab the YUYV logged information
        f_raw = fopen(sprintf('%s/yuyv_r_%s.log',log_dir,datestamp));
        yuyvMontage = fread(f_raw,Inf,'*uint32');
        yuyvMontage = reshape( ...
            yuyvMontage, ...
            [yuyvMeta{1}.w/2, yuyvMeta{1}.h, 1, numel(yuyvMeta)] ...
            );
        save(sprintf('%s/yuyv_%s.mat',log_dir,datestamp),'yuyvMontage');
    end
end