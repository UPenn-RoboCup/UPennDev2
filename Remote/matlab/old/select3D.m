function [  ] = select3D( h_omap, ~ )
%add_waypoint Add a waypoint
%   Should update any image handles, as well
    global WAYPOINTS H_WAYPOINTS;
    point = get(gca,'CurrentPoint');
    x = point(1,1);
    y = point(1,2);
  
    hold on;
    plot(x,y,'+')

    fprintf('Adding waypoint %.2f, %.2f\n',x,y);
    WAYPOINTS = [WAYPOINTS;[x,y]];
    set( H_WAYPOINTS,'XData', WAYPOINTS(:,1) );
    set( H_WAYPOINTS,'YData', WAYPOINTS(:,2) );

    drawnow;
end
