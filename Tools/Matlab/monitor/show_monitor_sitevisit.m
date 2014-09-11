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
        mesh_float(mesh_float>2.5) = 0;
%         mesh_float(mesh_float<0.1) = 0;

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
  
%             plot3(xs(i,:), ys(i,:), zs(i,:), '.');
%             hold on;
        end
        
        
        %%% Pixelize the points
        
        % Grid params: meters
        grid_res = 0.02; 
        
        xss = xs(:); yss = ys(:);  zss = zs(:);
        x_min = min(xss);  x_max = max(xss);
        y_min = min(yss);  y_max = max(yss);
        size_x = ceil((x_max - x_min) /grid_res);
        size_y = ceil((y_max - y_min) /grid_res);
        
        xis = ceil((xss - x_min) / grid_res);
        yis = ceil((yss - y_min) / grid_res);
        
        % check is max of xis/yis is exceeding grid size
        xis(xis>size_x) = size_x;
        xis(xis<=0) = 1;
        yis(yis>size_y) = size_y;
        yis(yis<=0) = 1;
        % Linear indices
        ind = sub2ind([size_x size_y], xis, yis);
        
        
        % TODO: dumb loop for now
        proj_plane = zeros(size_x, size_y);
        p_count = zeros(size_x, size_y);
        for i=1:length(ind)
            p_count(ind(i)) = p_count(ind(i)) + 1;
            proj_plane(ind(i)) = proj_plane(ind(i)) + zss(i);
        end
        
        % take average on the plane
        proj_plane = proj_plane ./ p_count;
        proj_plane(isnan(proj_plane(:))) = 0;
        
        % visual debug
        % imshow(proj_plane);

        
        proj_dz = proj_plane(1:end-1, :) - proj_plane(2:end,:);
        proj_dz = [abs(proj_dz); zeros(1, size(proj_dz, 2))];
        
        % visual debug
%         imshow(proj_dz);
        
        
        % Convert to uint8
        z_thres = 0.01; %TODO
        proj_dz(proj_dz<z_thres) = 1;
        proj_dz(proj_dz~=1) = 0;

        % visual debug
        imshow(proj_dz);

        % connected regions TODO
%         connected_regions(uint8(proj_dz), 1);
        
        
        % or we can first cluster (using hist or something) and check connected components
        % use edge detection and filter to a line? just a thought...
        
        
        hold off;
        
        
    end
  end
end
