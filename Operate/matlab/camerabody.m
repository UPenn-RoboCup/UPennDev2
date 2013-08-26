function ret=camerabody()
  global CAMERA

  CAMERA.head=[];
  CAMERA.left=[];
  CAMERA.right=[];

  CAMERA.head.width = 640;
  CAMERA.head.height = 360;
  CAMERA.left.width = 320;
  CAMERA.left.height = 240;
  CAMERA.right.width = 320;
  CAMERA.right.height = 240;

  CAMERA.init = @init;
  CAMERA.update_head = @update_head;
  CAMERA.update_left = @update_left;
  CAMERA.update_right = @update_right;

  ret = CAMERA;

  function init(a1,a2,a3)
    CAMERA.head.a = a1;
    CAMERA.left.a = a2;
    CAMERA.right.a = a3;

    set(a1,'XLim',.5+[0 CAMERA.head.width]);
    set(a1,'YLim',.5+[0 CAMERA.head.height]);
    set(a2,'XLim',.5+[0 CAMERA.left.width]);
    set(a2,'YLim',.5+[0 CAMERA.left.height]);
    set(a3,'XLim',.5+[0 CAMERA.right.width]);
    set(a3,'YLim',.5+[0 CAMERA.right.height]);

    CAMERA.head.image = image('Parent', CAMERA.head.a,'CData',[]); 
    CAMERA.left.image = image('Parent', CAMERA.left.a,'CData',[]); 
    CAMERA.right.image = image('Parent', CAMERA.right.a,'CData',[]); 

	  set(CAMERA.head.a, 'ButtonDownFcn', @camera_look);
	  set(CAMERA.left.a, 'ButtonDownFcn', @camera_look);
	  set(CAMERA.right.a, 'ButtonDownFcn', @camera_look);
  end

  function [nBytes] = update_head(fd)
    nBytes = 0;
    while udp_recv('getQueueSize',fd) > 0
      udp_data = udp_recv('receive',fd);
      nBytes = nBytes + numel(udp_data);
    end
    thor_camera_img = djpeg(udp_data);
    set(CAMERA.head.image,'Cdata', thor_camera_img);
  end

  function [nBytes] = update_left(fd)
    nBytes = 0;
    while udp_recv('getQueueSize',fd) > 0
      udp_data = udp_recv('receive',fd);
      nBytes = nBytes + numel(udp_data);
    end
    if numel( udp_data ) > 60
    data = add_jpeg_header(udp_data');
    
        thor_camera_img = djpeg(data);
        set(CAMERA.left.image,'Cdata', thor_camera_img);
    end
  end

  function [nBytes] = update_right(fd)
    nBytes = 0;
    while udp_recv('getQueueSize',fd) > 0
      udp_data = udp_recv('receive',fd);
      nBytes = nBytes + numel(udp_data);
    end
    if numel( udp_data ) > 60
    data = add_jpeg_header(udp_data');
    
        thor_camera_img = djpeg(data);
        set(CAMERA.right.image,'Cdata', thor_camera_img);
    end
  end

  function camera_look(h_omap, ~, flags)
    if h_omap==CAMERA.head.a 
      disp('center gaze')
    elseif h_omap==CAMERA.left.a 
      disp('left gaze')
    elseif h_omap==CAMERA.right.a 
      disp('right gaze')
    else
      return
    end

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

