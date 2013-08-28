function [ nBytes ] = update_sim_pose( pose )

    %UPDATE_OMAP Update the omap plot
    % Return the number of network bytes
    
    global H_POSE omap_invRes H_SLAM_WIDTH icon_scale icon_xs icon_ys omap_xmin omap_ymin;
    nBytes = 0;

    % Grab the data
    pose_data = msgpack('unpack',pose);
    pose_a = double(pose_data.a);
    pose_x = double(pose_data.x);
    pose_y = double(pose_data.y);
    new_omap_xmin = double(pose_data.omapXmin);
    new_omap_ymin = double(pose_data.omapYmin);
    
    
    
    xshift = new_omap_xmin - omap_xmin;
    yshift = new_omap_ymin - omap_ymin;
    
    if xshift~=0 || yshift~= 0
        set(H_POSE,'xticklabel',newlabels + xshift)
        set(H_POSE,'yticklabel',newlabels + yshift)
        omap_xmin = new_omap_xmin;
        omap_ymin = new_omap_ymin;
    end
    
    % TODO: omap_xmin, omap_ymin, omap_invRes, should be global, or with omap
    % TODO: They could just be zerod out in plotting, which could make things easy
    sa = sin(pose_a);
    ca = cos(pose_a);
    xi = (pose_x - omap_xmin) * omap_invRes;
    yi = (pose_y - omap_ymin) * omap_invRes;
    
    % Format for the icon
    xr = icon_xs*ca - icon_ys * sa + xi;
    yr = icon_xs*sa + icon_ys * ca + yi;
    % Set the data
    set(H_POSE,'XData',xr);
    set(H_POSE,'YData',yr);
    
    % Return the number of network bytes
    nBytes = 0;
end
