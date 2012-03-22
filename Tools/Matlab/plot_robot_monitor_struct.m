function h = plot_robot_monitor_struct(robot_struct,r_mon,scale)

teamColors = ['b', 'r'];

idColors = ['k', 'r', 'g', 'b'];
% Role: 0:Goalie / 1:Attack / 2:Defend / 3:Support
roleColors = ['m','g', 'k', 'y'];

hold on;

x0 = scale*[0 -.25 -.25];
y0 = scale*[0 -.10 +.10];
xm = mean(x0);
ym = mean(y0);

ca = cos(robot_struct.pose.a);
sa = sin(robot_struct.pose.a);

xr = x0*ca - y0*sa + robot_struct.pose.x;
yr = x0*sa + y0*ca + robot_struct.pose.y;

teamColors = ['b', 'r'];
hr = fill(xr, yr, teamColors(robot_struct.teamColor+1));

% disp attack bearing
xab = cos(robot_struct.attackBearing)*ca - sin(robot_struct.attackBearing)*sa;
yab = cos(robot_struct.attackBearing)*sa + sin(robot_struct.attackBearing)*ca;
ab_scale = 1.5;
quiver(robot_struct.pose.x, robot_struct.pose.y, xab*ab_scale,yab*ab_scale, 'k' );


x0 = robot_struct.pose.x;
y0 = robot_struct.pose.y;

%draw vision boundary
fov=r_mon.fov;
x1 = fov.TL(1)*ca - fov.TL(2)*sa + robot_struct.pose.x;
y1 = fov.TL(1)*sa + fov.TL(2)*ca + robot_struct.pose.y;

x2 = fov.TR(1)*ca - fov.TR(2)*sa + robot_struct.pose.x;
y2 = fov.TR(1)*sa + fov.TR(2)*ca + robot_struct.pose.y;
plot([x1 x2],[y1 y2], 'k--');
plot([x0 x1],[y0 y1], 'k--');
plot([x0 x2],[y0 y2], 'k--');




if ~isempty(robot_struct.ball) 
    if r_mon.ball.detect==1
      ball = [robot_struct.ball.x robot_struct.ball.y];
      xb = xr(1) + ball(1)*ca - ball(2)*sa;
      yb = yr(1) + ball(1)*sa + ball(2)*ca;
%    hb = plot(xb, yb, [idColors(robot_struct.id) 'o']);
      hb = plot(xb, yb, 'ro');
      plot([x0 xb],[y0 yb],'r');
      set(hb, 'MarkerSize', scale*4);
   end
end


    goal = r_mon.goal;
    if( goal.detect==1 )
        if(goal.color==2) % yellow
            marker = 'm';
        else
            marker = 'b';
        end
        marker2 = strcat(marker,'+');

        if( goal.v1.scale ~= 0 )
   	    x1 = goal.v1.x*ca - goal.v1.y*sa + robot_struct.pose.x;
   	    y1 = goal.v1.x*sa + goal.v1.y*ca + robot_struct.pose.y;
	    plot(x1,y1,marker2,'MarkerSize',12);
	    plot([x0 x1],[y0 y1],marker);
        end
        if( goal.v2.scale ~= 0 )
	    x2 = goal.v2.x*ca - goal.v2.y*sa + robot_struct.pose.x;
   	    y2 = goal.v2.x*sa + goal.v2.y*ca + robot_struct.pose.y;
	    plot(x2,y2,marker2,'MarkerSize',12);
	    plot([x0 x2],[y0 y2],marker);
        end
    end

    landmark = r_mon.landmark;
    if (landmark.detect==1)
	if (landmark.color==2) % yellow
	  marker='m';
        else
	  marker='b';
	end
        marker2 = strcat(marker,'x');

        x1 = landmark.v(1)*ca - landmark.v(2)*sa + robot_struct.pose.x;
        y1 = landmark.v(1)*sa + landmark.v(2)*ca + robot_struct.pose.y;
        plot(x1,y1,marker2,'MarkerSize',12);
        plot([x0 x1],[y0 y1],marker);
    end

hold off;

end
