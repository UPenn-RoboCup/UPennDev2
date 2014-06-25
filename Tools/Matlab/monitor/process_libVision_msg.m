function [needs_draw] = process_libVision_msg(metadata, raw, cam)
% Process each type of message
    msg_id = char(metadata.id);
    needs_draw = 0;
    
    if strcmp(msg_id,'detect')
        % Clear graphics objects
        % ball
        set(cam.p_ball,'Xdata', [],'Ydata', []);
        for i=1:3 
          % posts
          if i<3 
            set(cam.p_post{i},'Xdata', [],'Ydata', []);
          end
          % obstacles
          set(cam.h_obstacle{i}, 'Xdata', [], 'Ydata', []);
        end      
      
        % Set the debug information
        set(cam.a_debug, 'String', char(metadata.debug));
        % Process the ball detection result
        if isfield(metadata,'ball')
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
              
              rot = [cos(post_o) sin(post_o); -sin(post_o) cos(post_o)]';
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
          for i=1:min(3, numel(obstacles.iv))
            obs_c = obstacles.iv{i};
            % TODO: just a dummy rectangular for now
            w0 = 5;
            h0 = 15;
            x11 = obs_c + [w0 h0];
            x12 = obs_c + [-w0 h0];
            x21 = obs_c + [w0 -h0];
            x22 = obs_c + [-w0 -h0];
            obs_box = [x11; x12; x22; x21; x11];
            
            set(cam.h_obstacle{i}, 'XData', obs_box(:,1), 'YData', obs_box(:,2));
          end
        end
        
    elseif strcmp(msg_id,'world')
      if isfield(metadata, 'world')
        % msg_struct, vision_struct, scale, drawlevel, name
        drawlevel = 1;
        name = 'alvin';
        plot_robot(cam.h_field, metadata.world, [], 1.5, drawlevel, name);
        % Show messages
        set(cam.w_debug, 'String', char(metadata.world.info));
        needs_draw = 1;
      end

    elseif strcmp(msg_id,'head_camera')
        % Assume always JPEG
        cam.yuyv = djpeg(raw);
        set(cam.im_yuyv,'Cdata', cam.yuyv);
        % Set limits always, should not cost much CPU
        xlim(cam.f_yuyv,[0 metadata.w]);
        ylim(cam.f_yuyv,[0 metadata.h]);
        needs_draw = 1;
    elseif strcmp(msg_id,'labelA')
        cam.labelA = reshape(zlibUncompress(raw),[metadata.w,metadata.h])';
        set(cam.im_lA,'Cdata', cam.labelA);
        xlim(cam.f_lA,[0 metadata.w]);
        ylim(cam.f_lA,[0 metadata.h]);
        needs_draw = 1;
    elseif strcmp(msg_id,'labelB')
        cam.labelB = reshape(zlibUncompress(raw),[metadata.w,metadata.h])';
        set(cam.im_lB,'Cdata', cam.labelB);
        xlim(cam.f_lB,[0 metadata.w]);
        ylim(cam.f_lB,[0 metadata.h]);
        needs_draw = 1;
    end
end
