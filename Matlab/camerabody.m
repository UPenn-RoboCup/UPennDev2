function ret=camerabody()
    global CAMERA
    cam=[];
    
    cam.width = 640;
    cam.height = 360;    
    cam.init = @init;
    cam.update_head = @update_head;   

    ret = cam;

    function init(a1,a2,a3)
        CAMERA.a = a1;
        set(a1,'XLim',.5+[0 CAMERA.width]);
        set(a1,'YLim',.5+[0 CAMERA.height]);
        CAMERA.image = image('Parent', CAMERA.a,'CData',[]);                
        set(CAMERA.a, 'ButtonDownFcn', @camera_look);        
    end

    function [nBytes] = update_head(fd)
        nBytes = 0;
        while udp_recv('getQueueSize',fd) > 0
            udp_data = udp_recv('receive',fd);
            nBytes = nBytes + numel(udp_data);
        end        
        [metadata,offset] = msgpack('unpack',udp_data);
        cdepth = udp_data(offset+1:end);        
        if strncmp(char(metadata.c),'jpeg',3)==1
          set(CAMERA.image,'Cdata', djpeg(cdepth));
          set(CAMERA.a,'XLim',[1 metadata.width])
          set(CAMERA.a,'YLim',[1 metadata.height])
        end
    end

    function camera_look(h_omap, ~, flags)       
        
        %% Configuration variables
        h_fov = 60;
        v_fov = 60;
        cam_width = 320;
        cam_height = 240;
        cam_mid_x = cam_width/2;
        cam_mid_y = cam_height/2;
        focal_base = 320;
        focal_length = 260;
        gaze_data = {};
        gaze_data.cmd = 'unknown';
        if exist('flags')
            % Allow hcm directed gaze
        else
            % Add a waypoint
            point = get(gca,'CurrentPoint')
            x = point(1,1);
            y = cam_height-point(1,2);
            pan  = atan2( focal_length, x-cam_mid_x ) * 180/pi
            tilt = atan2( focal_length, y-cam_mid_y ) * 180/pi
            % Prep the command to send
            gaze_data.cmd = 'look';
            fprintf('Looking at %.2f, %.2f\n',x,y);
        end
        
        % Send the command
    end
end