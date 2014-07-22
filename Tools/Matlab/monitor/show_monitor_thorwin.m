function h = show_monitor_thorwin
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



%{
    cam.a_debug_ball = annotation('textbox',...
        [0.6 0 0.13 1],...
        'String','Top Camera',...
        'FontSize',10,...
        'FontName','Arial',...
        'LineStyle','--',...
        'EdgeColor',[1 1 0],...
        'LineWidth',2,...
        'BackgroundColor',[0.9  0.9 0.9],...
        'Color',[0.84 0.16 0]);

    cam.a_debug_goal = annotation('textbox',...
        [0.73 0 0.13 1],...
        'String','Top Camera',...
        'FontSize',10,...
        'FontName','Arial',...
        'LineStyle','--',...
        'EdgeColor',[1 1 0],...
        'LineWidth',2,...
        'BackgroundColor',[0.9  0.9 0.9],...
        'Color',[0.84 0.16 0]);
    cam.a_debug_obstacle = annotation('textbox',...
        [0.86 0 0.14 1],...
        'String','Top Camera',...
        'FontSize',10,...
        'FontName','Arial',...
        'LineStyle','--',...
        'EdgeColor',[1 1 0],...
        'LineWidth',2,...
        'BackgroundColor',[0.9  0.9 0.9],...
        'Color',[0.84 0.16 0]);
 
    % World debug
    cam.w_debug = annotation('textbox',...
        [0 0 0.3 0.5],...
        'String','Localization',...
        'FontSize',12,...
        'FontName','Arial'...
    );
%}  
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
    
    if strcmp(msg_id,'detect')
        % Clear graphics objects
        % ball
        set(cam.p_ball,'Xdata', [],'Ydata', []);
        %TODO: assume up to 3 obstacls for now
        for i=1:2 
          % posts
          set(cam.p_post{i},'Xdata', [],'Ydata', []);
          % obstacles
          set(cam.h_obstacle{i}, 'Xdata', [], 'Ydata', []);
        end      
      
        % Set the debug information
        set(cam.a_debug_ball, 'String', char(metadata.debug.ball));
        set(cam.a_debug_goal, 'String', char(metadata.debug.post));
        set(cam.a_debug_obstacle, 'String', char(metadata.debug.obstacle));



        % Process the ball detection result
        if isfield(metadata,'ball')
            %TODO: use ball t to remove old ball

            % Show our ball on the YUYV image plot
            % ball_c = metadata.ball.centroid * cam.scale;
            % ball_radius = (metadata.ball.axisMajor / 2) * cam.scale;
            % ball_box = [ball_c(1)-ball_radius ball_c(2)-ball_radius...
            %     2*ball_radius 2*ball_radius];
            % set(cam.p_ball, 'Xdata', ball_c(1));
            % set(cam.p_ball, 'Ydata', ball_c(2));
            % set(cam.r_ball, 'Position', ball_box);

            % Show ball on label image
            ball_c = metadata.ball.centroid;
            ball_radius = (metadata.ball.axisMajor / 2);
            ball_box = [ball_c(1)-ball_radius ball_c(2)-ball_radius...
                2*ball_radius 2*ball_radius];
            set(cam.p_ball, 'Xdata', ball_c(1));
            set(cam.p_ball, 'Ydata', ball_c(2));
            set(cam.r_ball, 'Position', ball_box);
        else
            %REMOVE BALL IF WE CANNOT SEE IT!
            set(cam.p_ball, 'Xdata', []);
            set(cam.p_ball, 'Ydata', []);
            set(cam.r_ball, 'Position', [0 0 0.0001 0.0001]);

        end
        if isfield(metadata,'posts')
            % Show on the plot
            % TODO: array of plot handles, for two goal posts
            for i=1:numel(metadata.posts)
              postStats = metadata.posts{i};
              
              % Plot on YUYV image
              % post_c = postStats.centroid * cam.scale;
              % w0 = postStats.axisMajor / 2 * cam.scale;
              % h0 = postStats.axisMinor / 2 * cam.scale;
              
              % Plot on label image
              post_c = postStats.post.centroid;
              w0 = postStats.post.axisMajor / 2;
              h0 = postStats.post.axisMinor / 2;
              post_o = postStats.post.orientation;
              
              rot = [cos(post_o) sin(post_o); -sin(post_o) cos(post_o)]'; %'
              x11 = post_c + [w0 h0] * rot;
              x12 = post_c + [-w0 h0] * rot;
              x21 = post_c + [w0 -h0] * rot;
              x22 = post_c + [-w0 -h0] * rot;
              post_box = [x11; x12; x22; x21; x11];
              % Draw
              set(cam.p_post{i}, 'XData', post_box(:,1), 'YData', post_box(:,2));
            end
        end
        
        if isfield(metadata, 'obstacles')
          obstacles = metadata.obstacles;
          for i=1:min(2, numel(obstacles.iv))
            obs_c = obstacles.iv{i};
            wo = obstacles.axisMajor(i)/2;
            ho = obstacles.axisMinor(i)/2;
            obs_o = obstacles.orientation(i);
            
            obs_rot = [cos(obs_o) sin(obs_o); -sin(obs_o) cos(obs_o)]';%'
            x11 = obs_c + [wo ho] * obs_rot;
            x12 = obs_c + [-wo ho] * obs_rot;
            x21 = obs_c + [wo -ho] * obs_rot;
            x22 = obs_c + [-wo -ho] * obs_rot;
            obs_box = [x11; x12; x22; x21; x11];

            set(cam.h_obstacle{i}, 'XData', obs_box(:,1), 'YData', obs_box(:,2));
          end
        end
        
        if isfield(metadata, 'line')
          %TODO: for now just plot one line?
          endpoint = metadata.line.endpoint{1};  %+0.5
          set(cam.h_line,'Xdata', [endpoint(1) endpoint(2)]);
          set(cam.h_line,'Ydata', [endpoint(3) endpoint(4)]);
        end
        
    elseif strcmp(msg_id,'world')
      if isfield(metadata, 'world')
        % msg_struct, vision_struct, scale, drawlevel, name
        drawlevel = 1;
        name = 'alvin';

        set(gcf,'CurrentAxes',cam.f_field);
        plot_robot(gca,metadata.world, [], 1.5, drawlevel, name);
        hold on;
        if isfield(metadata.world,'traj')
          num=metadata.world.traj.num;
          if num>0
            trajx = metadata.world.traj.x;
            trajy = metadata.world.traj.y;
            plot(trajx(1:num),trajy(1:num),'r');

            kickneeded = metadata.world.traj.kickneeded;            
            goal1 = metadata.world.traj.goal1;
            goal2 = metadata.world.traj.goal2;
            ballglobal = metadata.world.traj.ballglobal;
            ballglobal2 = metadata.world.traj.ballglobal2;
            ballglobal3 = metadata.world.traj.ballglobal3;

            plot([ballglobal(1) ballglobal2(1) ballglobal3(1)],...
                [ballglobal(2) ballglobal2(2) ballglobal3(2)],...
                'b','LineWidth',2);
            
            plot([ballglobal2(1) goal1(1)],[ballglobal2(2) goal1(2)],'k--');
            plot([ballglobal2(1) goal2(1)],[ballglobal2(2) goal2(2)],'k--');
          end
        end
        hold off;

        % Show messages
        set(cam.w_debug, 'String', char(metadata.world.info));
        needs_draw = 1;
      end

    elseif strcmp(msg_id,'head_camera')
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
    elseif strcmp(msg_id,'labelA')
        cam.labelA = reshape(zlibUncompress(raw),[metadata.w,metadata.h])';%'
        set(cam.im_lA,'Cdata', cam.labelA);
        xlim(cam.f_lA,[0 metadata.w]);
        ylim(cam.f_lA,[0 metadata.h]);
        needs_draw = 1;
    elseif strcmp(msg_id,'labelB')
        cam.labelB = reshape(zlibUncompress(raw),[metadata.w,metadata.h])';%'
        set(cam.im_lB,'Cdata', cam.labelB);
        xlim(cam.f_lB,[0 metadata.w]);
        ylim(cam.f_lB,[0 metadata.h]);
        needs_draw = 1;
    end
  end
end
