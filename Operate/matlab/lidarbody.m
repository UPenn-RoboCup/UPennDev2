function ret = lidarbody()
  global HEAD_LIDAR CHEST_LIDAR LIDAR H_FIGURE DEBUGMON POSE
  LIDAR.meshtype = 2; %Chest lidar default

  LIDAR.init = @init;
  LIDAR.update = @update;
  LIDAR.set_meshtype = @set_meshtype;
  LIDAR.wheel_calc = @wheel_calc;
  LIDAR.get_single_approach = @get_single_approach;
  LIDAR.get_double_approach = @get_double_approach;


  LIDAR.clear_points = @clear_points;
  LIDAR.set_zoomlevel = @set_zoomlevel;

  %depth map size and zooming
  LIDAR.xsize = 0;
  LIDAR.ysize = 0;
  LIDAR.xmag =0;
  LIDAR.ymag =0;

  LIDAR.last_posxy = [];

  LIDAR.selected_points = [];


  wdim_mesh=361;
  hdim_mesh=60;

  HEAD_LIDAR=[];
  HEAD_LIDAR.type = 0;
  HEAD_LIDAR.ranges=zeros(wdim_mesh,hdim_mesh);
  HEAD_LIDAR.range_actual=ones(wdim_mesh,hdim_mesh);

  HEAD_LIDAR.lidarangles={};
  HEAD_LIDAR.spineangles=[];
  HEAD_LIDAR.verts=[];
  HEAD_LIDAR.faces=[];
  HEAD_LIDAR.cdatas=[];
  HEAD_LIDAR.lidarrange = 1;
  HEAD_LIDAR.selected_points =[];
  HEAD_LIDAR.pointdraw = 0;

  HEAD_LIDAR.posex=[];
  HEAD_LIDAR.posey=[];
  HEAD_LIDAR.posea=[];


  CHEST_LIDAR=[];
  CHEST_LIDAR.type = 1;
  CHEST_LIDAR.ranges=zeros(wdim_mesh,hdim_mesh);
  CHEST_LIDAR.lidarangles={};
  CHEST_LIDAR.spineangles=[];
  CHEST_LIDAR.verts=[];
  CHEST_LIDAR.faces=[];
  CHEST_LIDAR.cdatas=[];
  CHEST_LIDAR.lidarrange = 1;
  CHEST_LIDAR.pointdraw = 0;

  CHEST_LIDAR.posex=[];
  CHEST_LIDAR.posey=[];
  CHEST_LIDAR.posea=[];

  function init(a1,a2,a_mesh)
    %maximum size for 3d mesh

    %% Setup the figure
    HEAD_LIDAR.p1 = a1;
    if a1~=0 
      axes(a1);
      HEAD_LIDAR.h1= imagesc( HEAD_LIDAR.ranges );
      set(HEAD_LIDAR.p1,'xtick',[],'ytick',[])
      colormap('HOT')
      set(HEAD_LIDAR.h1, 'ButtonDownFcn', {@select_3d,1});
      hold on;
		  HEAD_LIDAR.pointdraw = plot(a1,0,0,'g.');
      set(HEAD_LIDAR.pointdraw, 'MarkerSize', 40 );
      hold off;
    end

    CHEST_LIDAR.p1 = a2;
    if a2~=0 
      axes(a2);
      CHEST_LIDAR.h1= imagesc( CHEST_LIDAR.ranges );
      set(CHEST_LIDAR.p1,'xtick',[],'ytick',[])
      colormap('HOT')
      set(CHEST_LIDAR.h1, 'ButtonDownFcn', {@select_3d,2});
      hold on;
		  CHEST_LIDAR.pointdraw = plot(a2,0,0,'g*');
      hold off;
    end

    %Shared mesh display
    LIDAR.p = a_mesh;
    if a_mesh~=0 
      axes(a_mesh);
      LIDAR.h = patch('FaceColor','flat','EdgeColor','none',...
       'AmbientStrength',0.4,'SpecularStrength',0.9 );
      set(a_mesh,'xtick',[],'ytick',[], 'ztick',[])
      light('Position',[-3 1 3]);
      lighting flat
    end
  end

  function set_meshtype(h,~,meshtype)
    LIDAR.meshtype = meshtype;
    if meshtype==1
      set(LIDAR.h,'Faces',HEAD_LIDAR.faces);
      set(LIDAR.h,'Vertices',HEAD_LIDAR.verts);
      set(LIDAR.h,'FaceVertexCData',HEAD_LIDAR.cdatas);
    else
      set(LIDAR.h,'Faces',CHEST_LIDAR.faces);
      set(LIDAR.h,'Vertices',CHEST_LIDAR.verts);
      set(LIDAR.h,'FaceVertexCData',CHEST_LIDAR.cdatas);
    end
  end

  function [nBytes] = update(fd)
    disp('getting a mesh!')
    nBytes = 0;
    while udp_recv('getQueueSize',fd) > 0
      udp_data = udp_recv('receive',fd);
      nBytes = nBytes + numel(udp_data);
    end
    [metadata offset] = msgpack('unpack',udp_data);
    jdepth = udp_data(offset+1:end);
    if metadata.type==0
      HEAD_LIDAR.ranges = djpeg(jdepth);
      HEAD_LIDAR.lidarangles = metadata.lidarangles;
      HEAD_LIDAR.spineangles = metadata.spineangles;
      HEAD_LIDAR.lidarrange = metadata.lidarrange;
      HEAD_LIDAR.range0 = double(metadata.range0);
      HEAD_LIDAR.range1 = double(metadata.range1);

%{
      if isfield(metadata,'posex')
        HEAD_LIDAR.posex = cell2mat(metadata.posex);
        HEAD_LIDAR.posey = cell2mat(metadata.posey);
        HEAD_LIDAR.posea = cell2mat(metadata.posea);
      end
%}

      HEAD_LIDAR.verts=[];
      HEAD_LIDAR.faces=[];
      HEAD_LIDAR.cdatas=[];
%      disp('Head lidar data')
      update_lidar(0);

%TEST AUTODETECT
%	    detect_wheel(HEAD_LIDAR);

    else
      CHEST_LIDAR.ranges = djpeg(jdepth);
      CHEST_LIDAR.lidarangles = metadata.lidarangles;
      CHEST_LIDAR.spineangles = metadata.spineangles;
      CHEST_LIDAR.lidarrange = metadata.lidarrange;
      CHEST_LIDAR.range0 = double(metadata.range0);
      CHEST_LIDAR.range1 = double(metadata.range1);
%{
      if isfield(metadata,'posex')
        CHEST_LIDAR.posex = cell2mat(metadata.posex);
        CHEST_LIDAR.posey = cell2mat(metadata.posey);
        CHEST_LIDAR.posea = cell2mat(metadata.posea);
      end
%}
      CHEST_LIDAR.verts=[];
      CHEST_LIDAR.faces=[];
      CHEST_LIDAR.cdatas=[];
 %     disp('Chest lidar data')
      update_lidar(1);
    end
    set_meshtype(0,0,LIDAR.meshtype); % redraw figure
  end

  function data = wheel_calc()
    points3d = LIDAR.selected_points;
    data=[];
    if numel(points3d)>=3*3 
      leftrelpos = points3d(size(points3d,1)-2, :);
      rightrelpos = points3d(size(points3d,1)-1, :);
      toprelpos = points3d(size(points3d,1), :);
      if leftrelpos(1)>0.10 && rightrelpos(1) > 0.10 && toprelpos(1)>0.10 
        handlepos = (leftrelpos+rightrelpos) / 2;
        handleyaw = atan2(...
             leftrelpos(2)-rightrelpos(2),leftrelpos(1)-rightrelpos(1)...
              ) - pi/2;
        handlepitch = atan2(...
              toprelpos(1)-handlepos(1),toprelpos(3)-handlepos(3)...
              );
        handleradius = norm(leftrelpos-rightrelpos)/2;
        wheel_str=sprintf(...
            'Wheel property: pos %.2f %.2f %.2f yaw %.1f pitch %.1f radius %.2f',...
            handlepos(1),handlepos(2),handlepos(3),...
            handleyaw*180/pi,handlepitch*180/pi,handleradius );
				DEBUGMON.addtext(wheel_str);

        data = [handlepos handleyaw handlepitch handleradius];
%        send_control_packet('arm','wheelcalc',data);
		    HEAD_LIDAR.selected_points=[];
  		  set(HEAD_LIDAR.pointdraw,'XData',[]);
	  	  set(HEAD_LIDAR.pointdraw,'YData',[]);
        set(HEAD_LIDAR.pointdraw,'MarkerSize',30);

      end
    end
  end


%%%TODO
  function set_zoomlevel(h_omap, ~, flags)
    if flags==1 %zoom in
      DEBUGMON.addtext('Zoom in');
      LIDAR.xmag = LIDAR.xsize/2*0.75;
      LIDAR.ymag = LIDAR.ysize/2*0.75;
      if numel(LIDAR.last_posxy)>0
        zoom_in(LIDAR.last_posxy);
      else
        zoom_in([LIDAR.xsize/2+1,LIDAR.ysize/2+1]);
      end
    else
      DEBUGMON.addtext('Zoom out');
      LIDAR.xmag = LIDAR.xsize/2;
      LIDAR.ymag = LIDAR.ysize/2;
      set(HEAD_LIDAR.p1, 'XLim', [1 LIDAR.xsize]);
      set(HEAD_LIDAR.p1, 'YLim', [1 LIDAR.ysize]);

    end
  end

  function zoom_in(posxy)
    if LIDAR.xsize>0
      LIDAR.last_posxy = posxy;
      xMin = posxy(1)-LIDAR.xmag;
      yMin = posxy(2)-LIDAR.ymag;
      xMax = posxy(1)+LIDAR.xmag;
      yMax = posxy(2)+LIDAR.ymag;
      set(HEAD_LIDAR.p1, 'XLim', [xMin xMax]);
      set(HEAD_LIDAR.p1, 'YLim', [yMin yMax]);
    end
  end

  function clear_points(h_omap, ~, flags)
    HEAD_LIDAR.selected_points=[];
	  set(HEAD_LIDAR.pointdraw,'XData',[]);
 	  set(HEAD_LIDAR.pointdraw,'YData',[]);
    DEBUGMON.clearstr();
  end

  function targetpos = transform_global(relpos)
	  pose = POSE.pose_slam;
    targetpos=[];
    targetpos(1) =pose(1) + cos(pose(3)) * relpos(1) - sin(pose(3))*relpos(2);
    targetpos(2) =pose(2) + sin(pose(3)) * relpos(1) + cos(pose(3))*relpos(2);
  end

  function targetpos = get_single_approach()
	  points3d = LIDAR.selected_points;
    targetpos=[];
    if numel(points3d)>0
      targetrelpos = [points3d(size(points3d,1),1) 0];
      if targetrelpos(1) > 0.50 
          targetrelpos(1) = max(0,targetrelpos(1)-0.50);
          targetpos = transform_global(targetrelpos);
      end
    end
		LIDAR.clear_points();
  end

  function targetwp = get_double_approach()
    points3d = LIDAR.selected_points;
    targetwp=[];
    if numel(points3d)>=2*3 
      leftrelpos = points3d(size(points3d,1)-1, 1:2);
      rightrelpos = points3d(size(points3d,1), 1:2);

      if leftrelpos(1)>0.30 && rightrelpos(1)>0.30
        leftpos = transform_global(leftrelpos);
        rightpos = transform_global(rightrelpos);
        centerpos = (leftpos+rightpos)/2;
        angle = atan2(leftpos(2)-rightpos(2),leftpos(1)-rightpos(1)) - pi/2;
        targetpose= centerpos;
          %Waypoint generation for target pose

        r1 = - 0.60; 
        r2 = - 0.30; 

        curpose = POSE.pose
        targetpose1 = targetpose + [r1*cos(angle) r1*sin(angle)]
        targetpose2 = targetpose + [r2*cos(angle) r2*sin(angle)]
        targetwp =  reshape(  [targetpose1;targetpose2] ,[1 4]) ;
      end
    end
 		LIDAR.clear_points();
  end



  function select_3d(h_omap, ~, flags)

    clicktype = get(H_FIGURE,'selectionType');
    if strcmp(clicktype,'alt')>0
      point = get(gca,'CurrentPoint');
      posxy = [point(1,1) point(1,2)];
      zoom_in(posxy);
      return;
    end


    % Add a waypoint
    point = get(gca,'CurrentPoint');
    posxy = [point(1,1) point(1,2)];
    if flags==1 % HEAD LIDAR 
		  posxy
      HEAD_LIDAR.selected_points =[HEAD_LIDAR.selected_points;posxy];
      points=HEAD_LIDAR.selected_points
		  set(HEAD_LIDAR.pointdraw,'XData',HEAD_LIDAR.selected_points(:,1));
		  set(HEAD_LIDAR.pointdraw,'YData',HEAD_LIDAR.selected_points(:,2));

      if isfield(HEAD_LIDAR,'rayangles') % we have some data
  		  lidarangle_index = floor(posxy(2)-0.5);
	  	  rayangle_index = size(HEAD_LIDAR.rayangles,1)+1 - floor(posxy(1)-0.5);
  	    lidarangle_selected = HEAD_LIDAR.lidarangles_trimmed(lidarangle_index);
	  	  rayangle_selected = HEAD_LIDAR.rayangles(rayangle_index);
        range = HEAD_LIDAR.range_actual(rayangle_index,lidarangle_index);

        range = range+0.015; %to grab steering wheel
  		  endpos = lidartrans('headproject',...
	  			rayangle_selected, lidarangle_selected,range);
		    LIDAR.selected_points = [LIDAR.selected_points; endpos];
        disp_str = sprintf('Selected 3D pos: (%.3f %.3f %.3f)',endpos(1),endpos(2),endpos(3) );
        DEBUGMON.addtext(disp_str);
      else
      end
    end

  end



  ret= LIDAR;
end

