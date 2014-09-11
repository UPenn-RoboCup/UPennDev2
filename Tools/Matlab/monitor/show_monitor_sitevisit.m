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
        xs = bsxfun(@times, cos(s_angles)', bsxfun(@times, mesh, cos(v_angles)));
        ys = bsxfun(@times, sin(s_angles)', bsxfun(@times, mesh, cos(v_angles)));
        zs = -1*bsxfun(@times, mesh, sin(v_angles)) + metadata.lidarZ;
        
        % Body orientation
        body_pitch = metadata.bodyPitch;
        body_trans = [cos(body_pitch) sin(body_pitch); 
                      -sin(body_pitch)  cos(body_pitch)];
        
        
        % Visualization
        figure(2)
        xs_new = xs; zs_new = zs;
        for i = 1:n_scanlines 
            % TODO: better factorization
            new_xz = body_trans*[xs(1,:); zs(1,:)];
            xs_new(i,:) = new_xz(1,:);
            zs_new(i,:) = new_xz(2,:) + 1;  %TODO: bodyHeight

            plot3(xs_new(i,:), ys(i,:), zs_new(i,:), '.');
            hold on;
        end
        
        hold off;
        
        
    end
  end
end
