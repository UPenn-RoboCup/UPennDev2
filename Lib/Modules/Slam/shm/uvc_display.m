clear all;
top_dir = 'log_uvc/';
ts_list = dir( strcat(top_dir,'*.ts') );
%ts_order = 1:numel(ts_list);
ts_order = 1:5:numel(ts_list);

for uvc_log=ts_order
    
    ts_name = ts_list(uvc_log).name;
    fr_name = ts_name(1:end-3);
    fid = fopen( strcat(top_dir,ts_name) );
    if (fid~=-1)
        ts = fscanf(fid, '%f');
        fclose(fid);
        strcat(top_dir,fr_name);
        fid = fopen( strcat(top_dir,fr_name) );
        if (fid~=-1)
            yuyv_img = uint32( fread( fid, inf, 'uint32' ) );
            yuyv_img = reshape( yuyv_img, 640/2, 480 );
            fclose( fid );
            rgb_img = yuyv2rgb( yuyv_img );
            imagesc( imrotate(rgb_img,180) );
            title(uvc_log)
            pause(0.01)
            drawnow;
        end
    end
end