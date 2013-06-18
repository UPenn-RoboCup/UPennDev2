function ret=hmapbody()
  global HMAP POSE

  icon_scale = 40;
  icon_s = [0.25 0; -0.125 0.1; -0.125 -0.1]*icon_scale;

%   SLAM.waypoints_pixel=[];
%   SLAM.waypoints_xy=[];

  HMAP.init = @init;
  HMAP.update_hmap = @update_hmap;
%   SLAM.add_waypoint = @add_waypoint;
%   SLAM.get_waypoints = @get_waypoints;

%   SLAM.clear_waypoint = @clear_waypoint;
%   SLAM.start_movement = @start_movement;
  HMAP.update_pose = @update_pose;

  HMAP.hmap=[];
  HMAP.hmap.xmin = -10;
  HMAP.hmap.xmax = 10;
  HMAP.hmap.ymin = -10;
  HMAP.hmap.ymax = 10;
  HMAP.hmap.map_xsiz = 401;
  HMAP.hmap.map_ysiz = 401;

  HMAP.pose=[0 0 0];

  ret = HMAP;

  function init(a)
    HMAP.a = a;
    HMAP.clims = [ -0.1 4 ];
    HMAP.image =  imagesc( 'Parent', a, ...
    'XData', [1 HMAP.hmap.map_xsiz], 'YData', [1 HMAP.hmap.map_ysiz], ...
    'CData', []);
%     set(SLAM.image, 'ButtonDownFcn', @add_waypoint);

    hold on;
    HMAP.wayline = plot(a,0,0,'k*-' );
	  HMAP.pose_triangle = fill(...
			 icon_s(:,1), icon_s(:,2), 'g', 'LineWidth', 1);
    update_pose([0 0 0]);
    hold off;
    set(a,'XLim',.5+[1 HMAP.hmap.map_xsiz]);
    set(a,'YLim',.5+[1 HMAP.hmap.map_ysiz]);
  end



  function [nBytes] = update_hmap(fd)
    nBytes = 0;
    while udp_recv('getQueueSize',fd) > 0
      udp_data = udp_recv('receive',fd);
      nBytes = nBytes+numel(udp_data);
    end
    data_unpacked = msgpack('unpack',udp_data);
    thor_hmap = djpeg(data_unpacked.image);
    thor_hmapdata = data_unpacked.data;

	  HMAP.hmap.xmin = thor_hmapdata.Xmin;
	  HMAP.hmap.ymin = thor_hmapdata.Ymin;
	  HMAP.hmap.xmax = thor_hmapdata.Xmax;
	  HMAP.hmap.ymax = thor_hmapdata.Ymax;

    set(HMAP.image,'Cdata', thor_hmap);
    nBytes = numel(udp_data);
  end

  function update_pose(pose)
    HMAP.pose=pose;
    pixel_pos = transform_pixelpos([pose(1) pose(2)]);
    %rotate 90 degre to match the display orientation
    pos_transformed = rotz(icon_s, -pose(3)+pi/2) + repmat(pixel_pos,[3 1]);
	  set(HMAP.pose_triangle,...
			'XData',pos_transformed(:,1),'YData',pos_transformed(:,2) );
  end

  function ret = rotz(pos,angle)
    ca=cos(angle);
    sa=sin(angle);
    ret = [ca*pos(:,1)-sa*pos(:,2) sa*pos(:,1)+ca*pos(:,2)];
  end

  function pixelpos = transform_pixelpos(xy)
    xmag =(HMAP.hmap.map_xsiz-1)/2;
    ymag =(HMAP.hmap.map_ysiz-1)/2;
    centerpos = [xmag+1 ymag+1];
    xscale = xmag / (HMAP.hmap.xmax - HMAP.hmap.xmin) * 2;
    yscale = xmag / (HMAP.hmap.ymax - HMAP.hmap.ymin) * 2;
    centerxy = [ (HMAP.hmap.xmax + HMAP.hmap.xmin)/2 ...
								(HMAP.hmap.ymax + HMAP.hmap.ymin)/2 ];
    pixelposrel = [xy-centerxy].*[xscale yscale];
    pixelpos = centerpos + [pixelposrel(2) pixelposrel(1)];
  end

%   function xy = transform_pos(pixelpos)
%    %converts pixel pos to x,y pos in actual pose
%     xmag =(HMAP.hmap.map_xsiz-1)/2;
%     ymag =(HMAP.hmap.map_ysiz-1)/2;
%     centerpos = [xmag+1 ymag+1];
%     xscale = xmag / (HMAP.hmap.xmax - HMAP.hmap.xmin) * 2;
%     yscale = xmag / (HMAP.hmap.ymax - HMAP.hmap.ymin) * 2;
%     centerxy = [ (HMAP.hmap.xmax + HMAP.hmap.xmin)/2 ...
% 								(HMAP.hmap.ymax + HMAP.hmap.ymin)/2 ];
%     xy = centerxy + (pixelpos - centerpos)./[xscale yscale];
%     xy = [xy(2) xy(1)];
% 
% 
%   end
%   function clear_waypoint()
%     SLAM.waypoints_pixel=[];
%     SLAM.waypoints_xy=[];
%     set( SLAM.wayline, 'XData', [] );
%     set( SLAM.wayline, 'YData', [] );
%   end
%   function ret=get_waypoints()
%     ret=SLAM.waypoints_xy;
%   end
%   function add_waypoint(h_omap, ~, flags)
% 
%     % Add a waypoint
%     point = get(gca,'CurrentPoint');
%     posx = point(1,1); posy = point(1,2);
% 
%     xy = transform_pos([posx posy]);
% 
%     %Initial point start from current pose
%     if size(SLAM.waypoints_pixel,2)==0
%       pose = SLAM.pose;
%       pixel_pos = transform_pixelpos([pose(1) pose(2)]);
%       SLAM.waypoints_pixel=pixel_pos;
%     end
% 
%     SLAM.waypoints_pixel=[SLAM.waypoints_pixel;[posx,posy]];
%     SLAM.waypoints_xy=[SLAM.waypoints_xy;xy]; 
%     set( SLAM.wayline, 'XData', SLAM.waypoints_pixel(:,1) );
%     set( SLAM.wayline, 'YData', SLAM.waypoints_pixel(:,2) );
%     % Prep the command to send
% 
%     fprintf('Adding waypoint %.2f, %.2f\n',xy(1),xy(2));
%     drawnow;
%   end

end
