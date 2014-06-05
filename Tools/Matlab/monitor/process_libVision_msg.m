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
            set(cam.p_ball,'Xdata', ball_c(1));
            set(cam.p_ball,'Ydata', ball_c(2));
        else
            % Remove from the plot
            set(cam.p_ball,'Xdata', []);
            set(cam.p_ball,'Ydata', []);
        end
        if isfield(metadata,'posts')
            % Show on the plot
            metadata.posts
        else
            % Remove from the plot
        end
        
    elseif strcmp(msg_id,'world')

    elseif strcmp(msg_id,'yuyv')
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