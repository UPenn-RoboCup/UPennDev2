function [needs_draw] = process_libVision_msg(metadata, raw, cam)
% Process each type of message
    msg_id = char(metadata.id);
    needs_draw = 0;
    if strcmp(msg_id,'detect')
        % Set the debug information
        set(cam.a_debug, 'String', char(metadata.debug));
        % Process the ball detection result
        if isfield(metadata,'ball')
            % Show our ball on the YUYV image plot
            ball_c = metadata.ball.centroid * cam.scale;
            ball_radius = (metadata.ball.axisMajor / 2) * cam.scale;
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
            postStats = metadata.posts{1};
            post_c = postStats.centroid' * cam.scale;
            x0 = post_c(1);
            y0 = post_c(2);
            w0 = postStats.axisMajor * cam.scale / 2;
            h0 = postStats.axisMinor * cam.scale / 2;
            post_o = postStats.orientation;
            rot = [cos(post_o) sin(post_o); -sin(post_o) cos(post_o)];
            x11 = [x0 y0] + (rot*[w0 h0]')';
            x12 = [x0 y0] + (rot*[-w0 h0]')';
            x21 = [x0 y0] + (rot*[w0 -h0]')';
            x22 = [x0 y0] + (rot*[-w0 -h0]')';
            post_box = [x11; x12; x22; x21; x11];
            set(cam.p_post1, 'XData', post_box(:,1), 'YData', post_box(:,2));
        else
            % Remove from the plot
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