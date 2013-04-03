clear all;
top_dir = '~/shadwell/day2_third/log00_11_1c_01_07_79/';
fr_list = dir( strcat(top_dir,'fr_*') );
ts_list = dir( strcat(top_dir,'ts_*') );

use_rev = 0;
fr_order = 1:numel(fr_list);

for flir_log=fr_order
    flir_import2( strcat(top_dir,ts_list(flir_log).name) );
    fid = fopen( strcat(top_dir,fr_list(flir_log).name) );
    
    fr_order2 = 1:numel(flirtime);
    for i=fr_order2
        [read_img, cnt] = fread( fid, 320*256, 'uint16' );
        read_img = uint16(read_img);
        %[read_img, cnt] = fread( fid, 320*256, 'uint8' );
        %read_img = uint8(read_img);
        ir_img = reshape(read_img, 320, 256 )';
        imagesc( ir_img );
        title(sprintf('Frame %d, %f',frameid(i), unixtime(i) ));
        drawnow;
    end
    fclose( fid );
    
end