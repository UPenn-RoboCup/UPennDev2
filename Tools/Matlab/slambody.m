function ret=slambody()
  global SLAM POSE WAYPOINTS

  icon_scale = 40;
  icon_s = [0.25 0; -0.125 0.1; -0.125 -0.1]*icon_scale;


  icon_bbox = [0.50 0.64;
							 0.50 -0.64;
							-0.30 -0.64;
							-0.30 0.64; 
							0.50 0.64];


  redmap=zeros(256);
  bluemap=zeros(256);
  greenmap=zeros(256);

  th_obstacle = 80;
  th_wall = 250;
  redmap(1) = 0.1;
  bluemap(1) = 0.1;
  greenmap(1) = 0.1;

  for i=2:th_obstacle
    %Gray for known region
    redmap(i)=0;
    bluemap(i)=0;
    greenmap(i)=0;
  end

  for i=th_obstacle+1:th_wall
    %Red shade for obstacles
		ph = (i-th_obstacle)/(th_wall-th_obstacle);
    redmap(i)=0.5+0.5*ph;
	bluemap(i)=0;
    greenmap(i)=0.5*(1-ph);
  end

  for i=th_wall+1:256
    %white for wall
    redmap(i)=1;
    bluemap(i)=1;
	greenmap(i) = 1;
  end

  SLAM.waypoints_pixel=[];
  SLAM.waypoints_xy=[];

  SLAM.init = @init;
  SLAM.update_omap = @update_omap;
  SLAM.add_waypoint = @add_waypoint;
  SLAM.get_waypoints = @get_waypoints;
  SLAM.get_double_approach = @get_double_approach;


  SLAM.update_waypoints = @update_waypoints;


  SLAM.clear_waypoint = @clear_waypoint;
  SLAM.start_movement = @start_movement;
  SLAM.update_pose = @update_pose;
  SLAM.set_zoomlevel = @set_zoomlevel;

  SLAM.omap=[];
  SLAM.omap.xmin = -10;
  SLAM.omap.xmax = 10;
  SLAM.omap.ymin = -10;
  SLAM.omap.ymax = 10;
  SLAM.omap.map_xsiz = 401;
  SLAM.omap.map_ysiz = 401;

  SLAM.pose=[0 0 0];
  SLAM.slam_pose=[0 0 0];
  SLAM.torso_tilt = 0;
  SLAM.zoom_level = 0;
  SLAM.d_size = 200;

  ret = SLAM;

  function init(a)
    SLAM.a = a;
    SLAM.image =  imagesc( 'Parent', a, ...
    'XData', [1 SLAM.omap.map_xsiz], 'YData', [1 SLAM.omap.map_ysiz], ...
    'CData', ones(SLAM.omap.map_xsiz,SLAM.omap.map_ysiz,3) );
    set(SLAM.image, 'ButtonDownFcn', @add_waypoint);

    hold on;
    SLAM.wayline = plot(a,0,0,'g*-' );
	  SLAM.pose_triangle = plot(...
			 icon_s(:,1), icon_s(:,2), 'b', 'LineWidth', 1);

	  SLAM.pose_triangle_slam = fill(...
			 icon_s(:,1), icon_s(:,2), 'g', 'LineWidth', 1);

	  SLAM.bbox =  plot([0],[0],'b--', 'LineWidth', 1);

    update_pose([0 0 0],[0 0 0]);
    hold off;
    set(a,'XLim',.5+[1 SLAM.omap.map_xsiz]);
    set(a,'YLim',.5+[1 SLAM.omap.map_ysiz]);
  end

  function set_zoomlevel(h,~,d)
    disp('zoom level set')
    if d==1 
      SLAM.zoom_level = min(3,SLAM.zoom_level+1);
    else
      SLAM.zoom_level = max(0,SLAM.zoom_level-1);
    end

    d_size = 200;
    if SLAM.zoom_level==1 
      d_size = 150;
    elseif SLAM.zoom_level==2
      d_size = 100;
    elseif SLAM.zoom_level==3
      d_size = 50;
    end
    SLAM.d_size = d_size;
  end


  function [nBytes] = update_omap(fd)
    nBytes = 0;
    while udp_recv('getQueueSize',fd) > 0
      udp_data = udp_recv('receive',fd);
      nBytes = nBytes+numel(udp_data);
    end
    %data_unpacked = msgpack('unpack',udp_data);
    [data_unpacked, offset] = msgpack('unpack',udp_data);
    % new streaming
% disp(data_unpacked)


    cmap = udp_data(offset+1:end);
    if strncmp(char(data_unpacked.c),'jpeg',3)==1
        thor_omap = djpeg(cmap);
    else
        thor_omap = zlibUncompress(cmap);
        thor_omap = thor_omap';
    end
    thor_omapdata = data_unpacked.shift;

	  SLAM.omap.xmin = thor_omapdata.Xmin;
	  SLAM.omap.ymin = thor_omapdata.Ymin;
	  SLAM.omap.xmax = thor_omapdata.Xmax;
	  SLAM.omap.ymax = thor_omapdata.Ymax;
      
    pose_slam = data_unpacked.pose_slam;
    SLAM.torso_tilt = data_unpacked.torso_tilt;
    SLAM.update_pose(SLAM.pose, pose_slam);

%set(SLAM.image,'Cdata', thor_omap);

%now RGB color

    cmap_rgb = [];
    cmap_rgb(:,:,1) = redmap(thor_omap+1);
    cmap_rgb(:,:,2) = greenmap(thor_omap+1);
    cmap_rgb(:,:,3) = bluemap(thor_omap+1);
    set(SLAM.image,'Cdata', cmap_rgb);

    pose = SLAM.pose;
    pixel_pose = transform_pixelpos([pose(1) pose(2)]);

    xmin = max(1,pixel_pose(1)-SLAM.d_size);
    ymin = max(1,pixel_pose(2)-SLAM.d_size);
    xmax = min(SLAM.omap.map_xsiz, pixel_pose(1)+SLAM.d_size);
    ymax = min(SLAM.omap.map_ysiz, pixel_pose(2)+SLAM.d_size);

    set(SLAM.a,'XLim',.5+[xmin xmax]);
    set(SLAM.a,'YLim',.5+[ymin ymax]);

    nBytes = numel(udp_data);
  end

  function update_pose(pose,slam_pose)
    SLAM.pose=pose;
    pixel_pos = transform_pixelpos([pose(1) pose(2)]);
    %rotate 90 degre to match the display orientation
    pos_transformed = rotz(icon_s, -pose(3)+pi/2) + repmat(pixel_pos,[3 1]);
% 	  set(SLAM.pose_triangle,...
% 			'XData',pos_transformed(:,1),'YData',pos_transformed(:,2) );
    
    % Convert uint64 to double 
    slam_pose = double(slam_pose);

    % The y direction needs to be flipped
    SLAM.slam_pose=slam_pose;
    
    pixel_pos2 = transform_pixelpos([slam_pose(1) slam_pose(2)]);
    %rotate 90 degre to match the display orientation
    pos_transformed2 = rotz(icon_s, -slam_pose(3)+pi/2) + repmat(pixel_pos2,[3 1]);

	  set(SLAM.pose_triangle_slam,...
			'XData',pos_transformed2(:,1),'YData',pos_transformed2(:,2) );



    bbox_pos_transformed=[];
    icon_bbox_transformed = rotz(icon_bbox, slam_pose(3)) + ...
				repmat(SLAM.slam_pose(1:2), [size(icon_bbox,1) 1]);
    for i=1:size(icon_bbox,1)
       bbox_pos_transformed = [bbox_pos_transformed;
						transform_pixelpos(icon_bbox_transformed(i,:)  )];
    end
 	  set(SLAM.bbox,'XData',bbox_pos_transformed(:,1)...
				,'YData',bbox_pos_transformed(:,2));

  end

  function ret = rotz(pos,angle)
    ca=cos(angle);
    sa=sin(angle);
    ret = [ca*pos(:,1)-sa*pos(:,2) sa*pos(:,1)+ca*pos(:,2)];
  end

  function pixelpos = transform_pixelpos(xy)

    xmag =(SLAM.omap.map_xsiz-1)/2;
    ymag =(SLAM.omap.map_ysiz-1)/2;

    centerpos = [xmag+1 ymag+1];

    xscale = xmag / (SLAM.omap.xmax - SLAM.omap.xmin) * 2;
    yscale = xmag / (SLAM.omap.ymax - SLAM.omap.ymin) * 2;
    centerxy = [ (SLAM.omap.xmax + SLAM.omap.xmin)/2 ...
								(SLAM.omap.ymax + SLAM.omap.ymin)/2 ];
    if ~isfloat(xy)
        xy = double(xy);
    end
    pixelposrel = [xy-centerxy].*[xscale yscale];
    pixelpos = centerpos + [pixelposrel(2) pixelposrel(1)];
  end

  function xy = transform_pos(pixelpos)
    %converts pixel pos to x,y pos in actual pose
    xmag =(SLAM.omap.map_xsiz-1)/2;
    ymag =(SLAM.omap.map_ysiz-1)/2;
    centerpos = [xmag+1 ymag+1];

    xscale = xmag / (SLAM.omap.xmax - SLAM.omap.xmin) * 2;
    yscale = xmag / (SLAM.omap.ymax - SLAM.omap.ymin) * 2;
    centerxy = [ (SLAM.omap.xmax + SLAM.omap.xmin)/2 ...
								(SLAM.omap.ymax + SLAM.omap.ymin)/2 ];
    xy = centerxy + (pixelpos - centerpos)./[xscale yscale];
    xy = [xy(2) xy(1)];
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%new functions

  function update_waypoints(waypoints)   
    if size(waypoints,1)==0
       SLAM.waypoints_pixel=[];
       set( SLAM.wayline, 'XData', [] );
       set( SLAM.wayline, 'YData', [] ); 
    else
      waypoints=[SLAM.slam_pose(1) SLAM.slam_pose(2);waypoints];
      SLAM.waypoints_pixel=[];
      for i=1:size(waypoints,1)
        SLAM.waypoints_pixel=[SLAM.waypoints_pixel;
          transform_pixelpos(waypoints(i,:))];
      end
      set( SLAM.wayline, 'XData', SLAM.waypoints_pixel(:,1) );
      set( SLAM.wayline, 'YData', SLAM.waypoints_pixel(:,2) );
    end
  end

  function add_waypoint(h_omap, ~, flags)
    % Add a waypoint
    point = get(gca,'CurrentPoint');
    posx = point(1,1); posy = point(1,2);
    xy = transform_pos([posx posy]);
    WAYPOINTS.add_waypoint(xy)
  end

end
