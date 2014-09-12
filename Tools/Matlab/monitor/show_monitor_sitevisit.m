function h = show_monitor_sitevisit
  global cam 

  h = []
  h.init = @init;
  h.process_msg = @process_msg;

  



  function init()
    figure(1);
    clf;
    set(gcf,'position',[1 1 600 450]);
    f_mainA = gca;
    f_lA = axes('Units','Normalized','position',[0 0.6 0.3 0.4]);
    f_yuyv = axes('Units','Normalized','position',[0.3 0.6 0.3 0.4]);


    %    f_field = axes('Units','Normalized','position',[0.3 -0.05 0.3 0.5]);

    f_field = axes('Units','Normalized','position',[0 0 0.6 0.6]);

    cam = {};

    % Colormap for the labeled image
    cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
    cam.cmap = [cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];

    % LABELA
    set(gcf,'CurrentAxes',f_lA);
    im_lA = image(zeros(1));
    colormap(cam.cmap);
    hold on;
    p_ball = plot([0],[0], 'y*');
    % Remove from the plot
    set(p_ball,'Xdata', []);
    set(p_ball,'Ydata', []);
    r_ball = rectangle('Position', [0 0 1 1],... 
      'Curvature',[1,1], 'EdgeColor', 'b', 'LineWidth', 2);
    p_post = cell(2,1);
    for i=1:numel(p_post)
        p_post{i} = plot([0],[0], 'b-', 'LineWidth', 2);
        % Remove from the plot
        set(p_post{i},'Xdata', []);
        set(p_post{i},'Ydata', []);
    end
    % Assume up to 3 obstacles
    h_obstacle = cell(3,1);
    for i=1:numel(h_obstacle)
        h_obstacle{i} = plot([0],[0], 'r-', 'LineWidth', 2);
        % Remove from the plot
        set(h_obstacle{i},'Xdata', []);
        set(h_obstacle{i},'Ydata', []);
    end
    h_line = plot([0], [0], 'm--', 'LineWidth', 3);
    set(h_line, 'Xdata',[], 'Ydata', []);

    % yuyv
    set(gcf,'CurrentAxes',f_yuyv);
    im_yuyv = image(zeros(1));

    % Show the field here
    %set(gcf,'CurrentAxes',f_field);
    
    cam.h_field = f_field;
    hold on;
 
    % Camera 1 Debug messages


    set(gcf,'CurrentAxes',f_mainA);    
    cam.a_debug_ball=uicontrol('Style','text','Units','Normalized',...
       'Position',[0.6 0.3 0.13 0.7],'FontSize',10, ...
       'BackgroundColor',[0.9  0.9 0.9],...
        'FontName','Arial');

    cam.a_debug_goal=uicontrol('Style','text','Units','Normalized',...
       'Position',[0.73 0.3 0.13 0.7],'FontSize',10, ...
       'BackgroundColor',[0.9  0.9 0.9],...
        'FontName','Arial');

    cam.a_debug_obstacle=uicontrol('Style','text','Units','Normalized',...
       'Position',[0.86 0.3 0.14 0.7],'FontSize',10, ...
       'BackgroundColor',[0.9  0.9 0.9],...
        'FontName','Arial');

    cam.w_debug = uicontrol('Style','text','Units','Normalized',...
       'Position',[0.6 0 0.4 0.3],'FontSize',10, ...
       'BackgroundColor',[0.9  0.9 0.9],...
        'FontName','Arial');


 
    % Save the camera handles
    
    cam.f_lA = f_lA;
    cam.im_lA = im_lA;
    cam.f_yuyv = f_yuyv;
    cam.f_field = f_field;
    cam.im_yuyv = im_yuyv;
    cam.p_ball = p_ball;
    cam.r_ball = r_ball;
    cam.p_post = p_post;
    cam.h_obstacle = h_obstacle;    
    cam.h_line = h_line;
    % Plot scale
    % Default: labelA is half size, so scale twice
    scale = 2;
    cam.scale = 2;
  end


  function [needs_draw] = process_msg(metadata, raw, cam)
% Process each type of message
    msg_id = char(metadata.id);
    needs_draw = 0;
    

    if strcmp(msg_id,'head_camera')
        nr = 320;
        nc = 180;
        % Assume always JPEG
        cam.yuyv = djpeg(raw);
        yuyv_scaled = imresize(cam.yuyv, [nr nc]);
        set(cam.im_yuyv,'Cdata', yuyv_scaled);
        % Set limits always, should not cost much CPU
        xlim(cam.f_yuyv,[0 nc]);
        ylim(cam.f_yuyv,[0 nr]);
        needs_draw = 1;
    elseif strcmp(msg_id, 'mesh0')
        % metadata
        n_scanlines = metadata.n_scanlines;
        n_returns = metadata.n_returns;
        s_angles = metadata.a;
        % Raw
        mesh_float = typecast(raw, 'single');
        
        % clamp on ranges
        mesh_float(mesh_float>5) = 0;
        mesh_float(mesh_float<0.1) = 0;

        mesh = reshape(mesh_float, [n_returns n_scanlines])';
        % ray angles 
        rfov = floor(metadata.rfov / pi * 180);
        v_angles = rfov(1):0.25:rfov(2);        
        v_angles = v_angles / 180 * pi;
        
        if length(v_angles)>n_returns
            v_angles = v_angles(1:360);
        end
        

        
        % Convert to x, y, z
        xs0 = bsxfun(@times, cos(s_angles)', bsxfun(@times, mesh, cos(v_angles)));
        ys0 = bsxfun(@times, sin(s_angles)', bsxfun(@times, mesh, cos(v_angles)));
        zs0 = -1*bsxfun(@times, mesh, sin(v_angles)) + metadata.lidarZ;
        
        % Body orientation
        body_pitch = metadata.bodyPitch;
        body_trans = [cos(body_pitch) sin(body_pitch); 
                      -sin(body_pitch)  cos(body_pitch)];
        
        
        % Visualization
        figure(2)
        xs = xs0; ys = ys0; zs = zs0;
        for i = 1:n_scanlines 
            % TODO: better factorization
            new_xz = body_trans*[xs0(i,:); zs0(i,:)];
            xs(i,:) = new_xz(1,:);
            zs(i,:) = new_xz(2,:) + 1;  %TODO: bodyHeight
  
            plot3(xs(i,:), ys(i,:), zs(i,:), '.');
            hold on;
        end
        hold off;
        
                
        % Grid params: meters
        grid_res = 0.01; 
        
        xss = xs(:); yss = ys(:);  zss = zs(:);
        x_min = min(xss);  x_max = max(xss);
        y_min = min(yss);  y_max = max(yss);
        size_x = ceil((x_max - x_min) /grid_res);
        size_y = ceil((y_max - y_min) /grid_res);
        
        % Subscripts
        xis = ceil((xss - x_min) / grid_res);
        yis = ceil((yss - y_min) / grid_res);
        % Boundaries check
        xis(xis>size_x) = size_x;    xis(xis<=0) = 1;
        yis(yis>size_y) = size_y;    yis(yis<=0) = 1;
        
        % Linear indices
        ind = sub2ind([size_x size_y], xis, yis);
        
        
        % TODO: dumb loop for now
        proj_plane = zeros(size_x, size_y);
        p_count = zeros(size_x, size_y);
        for i=1:length(ind)
            p_count(ind(i)) = p_count(ind(i)) + 1;
            proj_plane(ind(i)) = proj_plane(ind(i)) + zss(i);
        end
        
        % Get rid of the body part
        p_count(1:0.2/grid_res, :) = 0;        
        
        thres1 = 0.6*max(p_count(:));
%         thres2 = mean(p_count(:));
        wall_ind = find(p_count(:)>thres1);
        
        hmap = zeros(size(p_count));
        hmap(wall_ind)=1;
        
        figure(3);
        imshow(hmap);
        
        
        % Retrieve the x, y info of the wall line
        [wall_xis wall_yis] = ind2sub([size_x size_y], wall_ind);
        % TODO: rounding u
        wall_xs = wall_xis*grid_res + x_min;
        wall_ys = wall_yis*grid_res + y_min;
        
        % Fit into a line
        % Use polyfit for now, should be easy to implement in lua later
        P = polyfit(wall_xis, wall_yis, 1);
        line_angle = atan(P(1));
        
        xi_c = mean(wall_xis);
        yi_c = mean(wall_yis);
        
        x1 = xi_c - 50*cos(line_angle);
        x2 = xi_c + 50*cos(line_angle);
        y1 = yi_c - 50*sin(line_angle);
        y2 = yi_c + 50*sin(line_angle);
        
        hold on;
        plot(yi_c, xi_c, 'y*')
        plot([y2 y1], [x2 x1],'r');
        hold off;

        
        % Right now just use the center points as target
        x_target = mean(wall_xs);
        y_target = mean(wall_ys);
               
        yaw_target = line_angle/pi*180 - 90;
        % TODO: filter angle into -pi/2, pi/2
        [x_target y_target yaw_target]
        
        
        
%         % Averaging the height
%         proj_plane = proj_plane ./ p_count;
%         % remove NaN and track free space
%         free_ind = isnan(proj_plane(:));
%         proj_plane(free_ind) = 0;
%         
%         
%         % visual debug
% %         imshow(proj_plane);
% %         colormap('hot');
% 
%         % Height difference
%         Jx = proj_plane(2:end, :) - proj_plane(1:end-1,:);
%         Jx = [ zeros(1, size(Jx, 2)); abs(Jx)];
%         
%         Jy = proj_plane(:,2:end) - proj_plane(:,1:end-1);
%         Jy = [zeros(size(Jy,1), 1) abs(Jy)];
%         
%         
        % visual debug
%         imshow(Jx);
        
        
        % For connecting horizontal planes.. seems not necessary for now
%         low_thes = 0.01; %TODO
%         proj_dz(free_ind) = 2;  % so that free space is not in consideration
%         proj_dz(proj_dz<low_thes) = 1;
%         proj_dz(proj_dz~=1) = 0;

%         % For finding vertical wall
%         high_thres = 0.03;
%         Jx(Jx>high_thres) = 1; Jx(Jx~=1) = 0;
%         Jy(Jy>high_thres) = 1; Jy(Jy~=1) = 0;
%         % Find 'cliffs'
% %         wall_ind = (Jx==1) | (Jy==1);
%         wall_ind = Jx==1;
%         p_walls = zeros(size(Jx));
%         p_walls(wall_ind) = 1;
        
%         % visual debug
%         imshow(p_walls);
% 
%         % connected regions: this doesn't give orientation...
%         props = connected_regions(uint8(p_walls), 1);
%         
%         for i=1:3
%             % plot the bounding box for debugging...
%             hold on;
%             bbox = props(i).boundingBox;
%             
%             bbox_xs = [bbox(1,1) bbox(2,1) bbox(2,1) bbox(1,1) bbox(1,1)];
%             bbox_ys = [bbox(1,2) bbox(1,2) bbox(2,2) bbox(2,2) bbox(1,2)];
%             
%             plot(bbox_ys, bbox_xs);
%         end
%         
%         hold off;

        
        
        % For vertical wall, we can cheat since we know its with 10cm-20cm
        % high...
        
        % use edge detection and filter to a line? something like line 
        % detection in lua... just a thought
                
        
    end
  end
end
