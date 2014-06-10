function [needs_draw] = process_libVision_msg(metadata, raw, cam)
% Process each type of message
    msg_id = char(metadata.id);
    needs_draw = 0;
    if strcmp(msg_id,'detect')
        % Set the debug information
        set(cam.a_debug, 'String', char(metadata.debug));
        % Process the ball detection result
        if isfield(metadata,'ball')
            %{ Show our ball on the YUYV image plot
            ball_c = metadata.ball.centroid * cam.scale;
            ball_radius = (metadata.ball.axisMajor / 2) * cam.scale;
            ball_box = [ball_c(1)-ball_radius ball_c(2)-ball_radius...
                2*ball_radius 2*ball_radius];
            set(cam.p_ball, 'Xdata', ball_c(1));
            set(cam.p_ball, 'Ydata', ball_c(2));
            set(cam.r_ball, 'Position', ball_box);
            %}

            % Show ball on label image
            ball_c = metadata.ball.centroid;
            ball_radius = (metadata.ball.axisMajor / 2);
            ball_box = [ball_c(1)-ball_radius ball_c(2)-ball_radius...
                2*ball_radius 2*ball_radius];
            set(cam.p_ball, 'Xdata', ball_c(1));
            set(cam.p_ball, 'Ydata', ball_c(2));
            set(cam.r_ball, 'Position', ball_box);

        else
            % Remove from the plot
            set(cam.p_ball,'Xdata', []);
            set(cam.p_ball,'Ydata', []);
        end
        if isfield(metadata,'posts')
            % Show on the plot
            % TODO: array of plot handles, for two goal posts
            for i=1:numel(metadata.posts)
              postStats = metadata.posts{i};

              %{ % Plot on YUYV image
              post_c = postStats.centroid * cam.scale;
              w0 = postStats.axisMajor / 2 * cam.scale;
              h0 = postStats.axisMinor / 2 * cam.scale;
              %}
              % Plot on label image
              post_c = postStats.centroid;
              w0 = postStats.axisMajor / 2;
              h0 = postStats.axisMinor / 2;

              post_o = postStats.orientation;
              rot = [cos(post_o) sin(post_o); -sin(post_o) cos(post_o)]';
              x11 = post_c + [w0 h0] * rot;
              x12 = post_c + [-w0 h0] * rot;
              x21 = post_c + [w0 -h0] * rot;
              x22 = post_c + [-w0 -h0] * rot;
              post_box = [x11; x12; x22; x21; x11];
              %TODO: make a struct of p_post
              if i==1
                set(cam.p_post1, 'XData', post_box(:,1), 'YData', post_box(:,2));
              elseif i==2
                set(cam.p_post2, 'XData', post_box(:,1), 'YData', post_box(:,2));
              end
            end
            
        else
            % Remove from the plot
            set(cam.p_post1,'Xdata', []);
            set(cam.p_post1,'Ydata', []);
        end
        
    elseif strcmp(msg_id,'world')

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
