function [needs_draw] = process_monitor_message(metadata, raw, cam)
% Process each type of message
    msg_id = char(metadata.id);
    needs_draw = 0;
    if strcmp(msg_id,'detect')
        % Process the ball detection result
        if isa(metadata.ball,'uint8')
            % Error message
            ball_debug = sprintf('Ball: %s',char(metadata.ball));
            % Remove from the plot
            set(cam.p_ball,'Xdata', []);
            set(cam.p_ball,'Ydata', []);
        else
            % Show our ball on the YUYV image plot
            ball_c = metadata.ball.centroid * cam.scale;
            set(cam.p_ball,'Xdata', ball_c(1));
            set(cam.p_ball,'Ydata', ball_c(2));
            ball_debug = sprintf('Ball: %f %f',metadata.ball.centroid);
        end
        set(cam.a_debug, 'String', ball_debug);
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