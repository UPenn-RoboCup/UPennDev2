function ret = lidarbody()
global HEAD_LIDAR CHEST_LIDAR LIDAR H_FIGURE DEBUGMON POSE CONTROL
LIDAR.init = @init;
LIDAR.update = @update;
LIDAR.set_meshtype = @set_meshtype;
LIDAR.set_img_display = @set_img_display;
LIDAR.wheel_calc = @wheel_calc;
LIDAR.get_single_approach = @get_single_approach;
LIDAR.get_double_approach = @get_double_approach;

% Which mesh to display
% Chest lidar default
LIDAR.meshtype = 2;

% 0: Show head
% 1: Show chest
LIDAR.mesh_img_display = 0;
% points draw on this handle
LIDAR.clicked_points  = []; % 2d
LIDAR.selected_points = []; % 3d
LIDAR.pointdraw = 0;

LIDAR.clear_points = @clear_points;
LIDAR.set_zoomlevel = @set_zoomlevel;

%depth map size and zooming
LIDAR.xsize = 0;
LIDAR.ysize = 0;
LIDAR.xmag =0;
LIDAR.ymag =0;

LIDAR.last_posxy = [];


wdim_mesh=361;
hdim_mesh=60;

HEAD_LIDAR=[];
HEAD_LIDAR.off_axis_height = 0.10; % 10cm off axis
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

    function init(a1,a_mesh)
        %maximum size for 3d mesh
        
        %% Setup the image figure
        if a1~=0
            LIDAR.p_img = a1;
            axes(a1);
            % Default to head lidar
            LIDAR.h_img = imagesc( HEAD_LIDAR.ranges );
            set(LIDAR.p_img,'xtick',[],'ytick',[])
            colormap('HOT')
            set(LIDAR.h_img, 'ButtonDownFcn', @select_3d );
            hold on;
            LIDAR.pointdraw = plot(a1,0,0,'g.');
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

    function set_img_display(h,~,val)
        LIDAR.mesh_img_display = 1-LIDAR.mesh_img_display;
        if LIDAR.mesh_img_display==0
            % Set the string to the next mesh (not current)
            set(h,'String','Chest');
        else
            set(h,'String','Head');
        end
        clear_points();
        draw_mesh_image();
    end

    function draw_mesh_image()
        % Draw the data
        if LIDAR.mesh_img_display==0
            %depthfig = flipdim(HEAD_LIDAR.ranges,2);
            depthfig = HEAD_LIDAR.ranges;
        elseif LIDAR.mesh_img_display==1
            depthfig = (CHEST_LIDAR.ranges)';
        end
        set(LIDAR.h_img,'Cdata', depthfig);
        set(LIDAR.p_img, 'XLim', [1 size(depthfig,2)]);
        set(LIDAR.p_img, 'YLim', [1 size(depthfig,1)]);
    end

    function [nBytes] = update(fd)
        disp('Getting a mesh!');
        t0 = tic;
        nBytes = 0;
        while udp_recv('getQueueSize',fd) > 0
            udp_data = udp_recv('receive',fd);
            nBytes = nBytes + numel(udp_data);
        end
        [metadata,offset] = msgpack('unpack',udp_data);
        disp(metadata)
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
            HEAD_LIDAR.depths = metadata.depths;
            % Update the figure
            if LIDAR.mesh_img_display==0
                draw_mesh_image()
            end
        else
            % Save data
            CHEST_LIDAR.ranges = depth_img;
            CHEST_LIDAR.fov_angles = fov_angles;
            CHEST_LIDAR.scanline_angles = scanline_angles;
            CHEST_LIDAR.depths = metadata.depths;
            % Update the figure
            if LIDAR.mesh_img_display==1
                draw_mesh_image()
            end
        end

        % end of update
        tPassed = toc(t0);
        fprintf('Update lidar: %f seconds.\n',tPassed);
    end

    function data = wheel_calc(h,~,val)
        points3d = LIDAR.selected_points
        data=[];
        disp('Wheel calculation...');
        if numel(points3d)>=3*3

            % NOTE: Assume a left right top clicking order!!
            leftrelpos  = points3d(size(points3d,1)-2, :);
            rightrelpos = points3d(size(points3d,1)-1, :);
            toprelpos   = points3d(size(points3d,1),   :);

            % Find the center of the wheel
            handlepos = (leftrelpos+rightrelpos) / 2;
            if handlepos(1) > 1 || handlepos(1) < 0.10
                % x distance in meters
                disp('Handle is too far or too close!');
                disp(handlepos)
                %return;
            end

            % Find the radius of the wheel
            handleradius = norm(leftrelpos-rightrelpos)/2;
            if handleradius>1 || handleradius<0.10
                % radius in meters
                disp('Radius is too big or too small!');
                disp(handleradius)
                %return;
            end

            handleyaw = atan2(...
                leftrelpos(2)-rightrelpos(2), ...
                leftrelpos(1)-rightrelpos(1)) ...
                - pi/2;
            % TODO: yaw checks

            handlepitch = atan2(...
                toprelpos(1)-handlepos(1),toprelpos(3)-handlepos(3)...
                );
            % TODO: pitch checks
            
            % Debug message
            wheel_str = sprintf(...
                'Wheel property: pos %.2f %.2f %.2f yaw %.1f pitch %.1f radius %.2f',...
                handlepos(1),handlepos(2),handlepos(3),...
                handleyaw*180/pi,handlepitch*180/pi,handleradius );
            DEBUGMON.addtext(wheel_str);
            
            % Overwrite wheel estimate?
            % TODO: use two separate wheel estimates?
            % TODO: send this data to the robot
            LIDAR.wheel_model = [handlepos handleyaw handlepitch handleradius]
            CONTROL.send_control_packet([],[],'hcm','wheel','model',LIDAR.wheel_model);

            % Reset the user clicked points
            LIDAR.clicked_points = [];
            set(LIDAR.pointdraw,'XData',[]);
            set(LIDAR.pointdraw,'YData',[]);
            set(LIDAR.pointdraw,'MarkerSize',30);

            % TODO: Draw another point on there, with the actual wheel center?
            
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
        LIDAR.clicked_points  = []; % 2d
        LIDAR.selected_points = []; % 3d
        set(LIDAR.pointdraw,'XData',[]);
        set(LIDAR.pointdraw,'YData',[]);
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
        posxy = [point(1,1) point(1,2)]

        % Plot all the 2d points
        LIDAR.clicked_points = [LIDAR.clicked_points;posxy];
            set(LIDAR.pointdraw,'XData',LIDAR.clicked_points(:,1));
            set(LIDAR.pointdraw,'YData',LIDAR.clicked_points(:,2));
        
        % if head
        if LIDAR.mesh_img_display==0
            fov_angle_index = numel(HEAD_LIDAR.fov_angles)+1 - floor(posxy(1)-0.5);
            scanline_index = floor(posxy(2)-0.5);
            range = double( HEAD_LIDAR.ranges(scanline_index,fov_angle_index) )
            range = (range/255.0) * (HEAD_LIDAR.depths(2)-HEAD_LIDAR.depths(1));
            range = range + HEAD_LIDAR.depths(1)
            fov_angle_selected = -1*HEAD_LIDAR.fov_angles(fov_angle_index)
            scanline_angle_selected = -1*HEAD_LIDAR.scanline_angles(scanline_index)
            % TODO: Average nearby neighbor ranges
            
            global_point = lidartrans(...
                'headproject', ...
                fov_angle_selected, ...
                scanline_angle_selected, ...
                range)
            
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
            
            LIDAR.selected_points = [LIDAR.selected_points; global_point(1:3)'];
            disp_str = sprintf('Selected (%.3f %.3f %.3f)', ...
                    global_point(1),global_point(2),global_point(3) );
            DEBUGMON.addtext(disp_str);
        end % head
    end % select_3d

ret = LIDAR;
end