function ret = lidarbody()
global HEAD_LIDAR CHEST_LIDAR LIDAR H_FIGURE DEBUGMON POSE SLAM CONTROL
LIDAR.init = @init;
LIDAR.update = @update;
LIDAR.set_meshtype = @set_meshtype;
LIDAR.set_img_display = @set_img_display;
LIDAR.get_depth_img = @get_depth_img;
LIDAR.get_single_approach = @get_single_approach;
LIDAR.get_double_approach = @get_double_approach;
LIDAR.clear_points = @clear_points;
LIDAR.set_zoomlevel = @set_zoomlevel;

% Which mesh to display: 0-HEAD, 1-CHEST
% Chest lidar default
LIDAR.meshtype = 1;
LIDAR.mesh_cnt = 0;

% 0: Show head
% 1: Show chest
LIDAR.depth_img_display = 0;
% points draw on this handle
LIDAR.clicked_points  = []; % 2d
LIDAR.selected_points = []; % 3d
LIDAR.pointdraw = 0;

%depth map size and zooming
LIDAR.xsize = 0;
LIDAR.ysize = 0;
LIDAR.xmag =0;
LIDAR.ymag =0;

LIDAR.last_posxy = [];


wdim_mesh=361;
hdim_mesh=60;

HEAD_LIDAR=[];
HEAD_LIDAR.off_axis_height = 0.01; % 1cm off axis
HEAD_LIDAR.neck_height = 0.30;
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

HEAD_LIDAR.posex=[];
HEAD_LIDAR.posey=[];
HEAD_LIDAR.posea=[];


CHEST_LIDAR=[];
CHEST_LIDAR.off_axis_depth = 0.05;
CHEST_LIDAR.chest_depth  = 0.05;
CHEST_LIDAR.chest_height = 0.10;
CHEST_LIDAR.type = 1;
CHEST_LIDAR.ranges=zeros(wdim_mesh,hdim_mesh);
CHEST_LIDAR.lidarangles={};
CHEST_LIDAR.spineangles=[];
CHEST_LIDAR.verts=[];
CHEST_LIDAR.faces=[];
CHEST_LIDAR.cdatas=[];
CHEST_LIDAR.lidarrange = 1;

CHEST_LIDAR.posex=[];
CHEST_LIDAR.posey=[];
CHEST_LIDAR.posea=[];

    function init(a_depth,a_mesh)
        %maximum size for 3d mesh
        
        %% Setup the image figure
        if a_depth~=0
            LIDAR.p_img = a_depth;
            axes(a_depth);
            % Default to head lidar
            LIDAR.h_img = imagesc( HEAD_LIDAR.ranges );
            set(LIDAR.p_img,'xtick',[],'ytick',[])
            colormap('JET')
            set(LIDAR.h_img, 'ButtonDownFcn', @select_3d );
            hold on;
            LIDAR.pointdraw = plot(a_depth,0,0,'g.');
            set(LIDAR.pointdraw, 'MarkerSize', 40 );
            hold off;
        end
        
        % Shared 3d mesh display
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

    function set_meshtype(~, ~, meshtype)
        LIDAR.meshtype = meshtype;
        % Only use chest lidar for mesh
        if meshtype == 1
          deg2rad = pi/180;
          CONTROL.send_control_packet([],[],...
              'vcm','chest_lidar','scanlines',[-45*deg2rad, 45*deg2rad, 1/deg2rad]);
          CONTROL.send_control_packet([],[],'vcm','chest_lidar','depths',[.1,5]);
          CONTROL.send_control_packet([],[],'vcm','chest_lidar','net',[2,2,40]);
          % LIDAR update rate: 40 Hz
        end

        update_mesh_display();
    end

    function update_mesh_display()
        if LIDAR.meshtype==0
            set(LIDAR.h,'Faces',HEAD_LIDAR.faces);
            set(LIDAR.h,'Vertices',HEAD_LIDAR.verts);
            set(LIDAR.h,'FaceVertexCData',HEAD_LIDAR.cdatas);
        else
            set(LIDAR.h,'Faces',CHEST_LIDAR.faces);
            set(LIDAR.h,'Vertices',CHEST_LIDAR.verts);
            set(LIDAR.h,'FaceVertexCData',CHEST_LIDAR.cdatas);
        end
    end

    function set_img_display(h,~,val)
        LIDAR.depth_img_display = 1-LIDAR.depth_img_display;
        if LIDAR.depth_img_display==0
            % Set the string to the next mesh (not current)
            set(h,'String','Chest');
        else
            set(h,'String','Head');
        end
        clear_points();
        draw_depth_image();
    end

    function get_depth_img(h,~)
        if LIDAR.depth_img_display==0
            % head
            CONTROL.send_control_packet([],[],'vcm','head_lidar','depths',[.1,2]);
            CONTROL.send_control_packet([],[],'vcm','head_lidar','net',[1,1,0]);
        else
            % chest
            CONTROL.chest_depth = true;
            CONTROL.send_control_packet([],[],'vcm','chest_lidar','depths',[.1,5]);
            CONTROL.send_control_packet([],[],'vcm','chest_lidar','net',[1,1,0]);
        end
    end

    function draw_depth_image()
        % Draw the data
        if LIDAR.depth_img_display==0
            set(LIDAR.h_img,'Cdata', HEAD_LIDAR.ranges);
            set(LIDAR.p_img, 'XLim', [1 size(HEAD_LIDAR.ranges,2)]);
            set(LIDAR.p_img, 'YLim', [1 size(HEAD_LIDAR.ranges,1)]);
        else
            set(LIDAR.h_img,'Cdata', CHEST_LIDAR.ranges);
            set(LIDAR.p_img, 'XLim', [1 size(CHEST_LIDAR.ranges,2)]);
            set(LIDAR.p_img, 'YLim', [1 size(CHEST_LIDAR.ranges,1)]);
        end
    end
        
    function [nBytes] = update(fd)
        t0 = tic;
        nBytes = 0;
        while udp_recv('getQueueSize',fd) > 0
            udp_data = udp_recv('receive',fd);
            nBytes = nBytes + numel(udp_data);
        end
        [metadata,offset] = msgpack('unpack',udp_data);
        %disp(metadata)
        cdepth = udp_data(offset+1:end);
        if strncmp(char(metadata.c),'jpeg',3)==1
            depth_img = djpeg(cdepth);
        else
            depth_img = zlibUncompress(cdepth);
            depth_img = reshape(depth_img,[metadata.resolution(2) metadata.resolution(1)]);
            depth_img = depth_img';
        end
        % Calculate the angles
        fov_angles = metadata.fov(1) : .25*(pi/180) : metadata.fov(2);
        scanline_angles = metadata.scanlines(1) : 1/metadata.scanlines(3) : metadata.scanlines(2);
        
        if strncmp(char(metadata.name),'head',3)==1
            % Save data
            HEAD_LIDAR.ranges = depth_img;
            HEAD_LIDAR.fov_angles = fov_angles;
            HEAD_LIDAR.scanline_angles = scanline_angles;
            HEAD_LIDAR.depths = double(metadata.depths);
            % Update the figures
            draw_depth_image();
            update_mesh(0);
            update_mesh_display();
        else
            LIDAR.mesh_cnt = LIDAR.mesh_cnt + 1;
            % Save data
            CHEST_LIDAR.ranges = depth_img';
            CHEST_LIDAR.fov_angles = fov_angles;
            CHEST_LIDAR.scanline_angles = scanline_angles;
            CHEST_LIDAR.depths = double(metadata.depths);
            % Update depth image
            draw_depth_image();
            % Update mesh image
            update_mesh(1);
            if mod(LIDAR.mesh_cnt, 10) == 0
                update_mesh_display();
            end
        end
        
        % end of update
        tPassed = toc(t0);
%         fprintf('Update lidar: %f seconds.\n',tPassed);
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
        LIDAR.clicked_points  = []; % 2d
        LIDAR.selected_points = []; % 3d
        set(LIDAR.pointdraw,'XData',[]);
        set(LIDAR.pointdraw,'YData',[]);
        set(LIDAR.pointdraw,'MarkerSize',30);
        DEBUGMON.clearstr();
    end

    function targetpos = transform_global(relpos)
        %pose = POSE.pose_slam;
        pose = SLAM.slam_pose;
        targetpos=[];
        targetpos(1) =pose(1) + cos(pose(3)) * relpos(1) - sin(pose(3))*relpos(2);
        targetpos(2) =pose(2) + sin(pose(3)) * relpos(1) + cos(pose(3))*relpos(2);
        targetpos(3) = relpos(3);
        
        % Body Tilt
        ca = cos(SLAM.torso_tilt);
        sa = sin(SLAM.torso_tilt);
        targetpos(1) = targetpos(1)*ca + targetpos(3)*sa;
        targetpos(3) = -targetpos(1)*sa + targetpos(3)*ca;
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
            % 3D
            leftrelpos = points3d(size(points3d,1)-1, :);
            rightrelpos = points3d(size(points3d,1), :);
             
            if leftrelpos(1)>0.30 && rightrelpos(1)>0.30
                leftpos = transform_global(leftrelpos);
                rightpos = transform_global(rightrelpos);
                centerpos = (leftrelpos + rightrelpos)/2;
                centerpos(1) = max(0, centerpos(1) - 0.3);
                angle = atan2(leftpos(2)-rightpos(2),leftpos(1)-rightpos(1)) - pi/2;
                targetwp = [centerpos(1), centerpos(2), angle];
                
                %{
                r1 = - 0.60;
                r2 = - 0.30;
                
                curpose = POSE.pose
                targetpose1 = targetpose + [r1*cos(angle) r1*sin(angle)]
                targetpose2 = targetpose + [r2*cos(angle) r2*sin(angle)]
                targetwp =  reshape(  [targetpose1;targetpose2] ,[1 4]) ;
                %}
            end
        end
        LIDAR.clear_points();
    end

    function select_3d(~, ~, flags)
        
        clicktype = get(H_FIGURE,'selectionType');
        if strcmp(clicktype,'alt')>0
            point = get(gca,'CurrentPoint');
            posxy = [point(1,1) point(1,2)];
            zoom_in(posxy);
            return;
        end
        
        % Add a circle point
        point = get(gca,'CurrentPoint');
        posxy = [point(1,1) point(1,2)];
        

        % Plot all the 2d points
        LIDAR.clicked_points = [LIDAR.clicked_points;posxy];
            set(LIDAR.pointdraw,'XData',LIDAR.clicked_points(:,1));
            set(LIDAR.pointdraw,'YData',LIDAR.clicked_points(:,2));
        
        % if head
        if LIDAR.depth_img_display==0
            % Round the index to an integer
            fov_angle_index = round( posxy(1) );
            scanline_index  = round( posxy(2) );
            % grab the range
            range = HEAD_LIDAR.ranges(scanline_index,fov_angle_index)
            range = double(range)/255 * (HEAD_LIDAR.depths(2)-HEAD_LIDAR.depths(1));
            range = range + HEAD_LIDAR.depths(1);
            fov_angle_selected = -1*HEAD_LIDAR.fov_angles(fov_angle_index);
            scanline_angle_selected = HEAD_LIDAR.scanline_angles(scanline_index);
            % TODO: Average nearby neighbor ranges
            % Convert the local coordinates to global
            local_point = [ ...
            range*cos(fov_angle_selected); ...
            range*sin(fov_angle_selected); ...
            HEAD_LIDAR.off_axis_height; ...
            0
            ];
            local_to_global = eye(4);
            local_to_global(1,1) = cos(scanline_angle_selected);
            local_to_global(3,3) = cos(scanline_angle_selected);
            local_to_global(1,3) = sin(scanline_angle_selected);
            local_to_global(3,1) = -sin(scanline_angle_selected);
            global_point = local_to_global * local_point;
            global_point(3) = global_point(3) + HEAD_LIDAR.neck_height;
            
        else
            % Round the index to an integer
            fov_angle_index = round( posxy(2) );
            scanline_index  = round( posxy(1) );
            % grab the range
            range = CHEST_LIDAR.ranges(fov_angle_index,scanline_index);
            range = double(range)/255 * (CHEST_LIDAR.depths(2)-CHEST_LIDAR.depths(1))
            range = range + CHEST_LIDAR.depths(1);
            % Grab the correct angles
            fov_angle_selected = -1*CHEST_LIDAR.fov_angles(fov_angle_index)
            scanline_angle_selected = -1*CHEST_LIDAR.scanline_angles(scanline_index);
            % TODO: Average nearby neighbor ranges
            %{
            global_point = lidartrans(...
                'headproject', ...
                fov_angle_selected, ...
                scanline_angle_selected, ...
                range);
            %}
            
            local_point = [ ...
            range*cos(fov_angle_selected)+CHEST_LIDAR.off_axis_depth; ...
            0
            range*sin(fov_angle_selected); ...
            0
            ];
            local_to_global = eye(4);
            local_to_global(1,1) = cos(scanline_angle_selected);
            local_to_global(2,2) = cos(scanline_angle_selected);
            local_to_global(1,2) = sin(scanline_angle_selected);
            local_to_global(2,1) = -sin(scanline_angle_selected);
            global_point = local_to_global * local_point;
            global_point(1) = global_point(1) + CHEST_LIDAR.chest_depth;
            global_point(3) = global_point(3) + CHEST_LIDAR.chest_height;
        end % head/chest
        LIDAR.selected_points = [LIDAR.selected_points; global_point(1:3)'];
        disp_str = sprintf('Selected (%.3f %.3f %.3f)', ...
                global_point(1),global_point(2),global_point(3) );
        DEBUGMON.addtext(disp_str);
    end % select_3d

ret = LIDAR;
end
