function h = plot_robot_monitor_struct(robot_struct,r_mon,scale)
%This function shows the robot over the field map

  x0 = robot_struct.pose.x;
  y0 = robot_struct.pose.y;
  ca = cos(robot_struct.pose.a);
  sa = sin(robot_struct.pose.a);
   
  hold on;
  plot_robot(robot_struct,scale);
%  plot_ball(robot_struct.ball);
  plot_ball_mon(r_mon.ball,scale);
  plot_fov(r_mon.fov);
  plot_goal(r_mon.goal,scale);
  plot_landmark(r_mon.landmark,scale);
  hold off;

  %subfunctions

  function plot_robot(robot,scale)
    xRobot = [0 -.25 -.25]*2/scale;
    yRobot = [0 -.10 +.10]*2/scale;
    xr = xRobot*ca - yRobot*sa + x0;
    yr = xRobot*sa + yRobot*ca + y0;
    xm = mean(xRobot);
    ym = mean(yRobot);

    teamColors = ['b', 'r'];
    idColors = ['k', 'r', 'g', 'b'];
    % Role:  1:Attack / 2:Defend / 3:Support / 4: Goalie
    roleColors = {'m','k', 'k--','g'};

    teamColors = ['b', 'r'];
    hr = fill(xr, yr, teamColors(robot.teamColor+1));

    if robot.role>1 
      h_role=plot([xr xr(1)],[yr yr(1)],roleColors{robot.role});
      set(h_role,'LineWidth',3);
    end

    % disp attack bearing
    xab = cos(robot.attackBearing)*ca - sin(robot.attackBearing)*sa;
    yab = cos(robot.attackBearing)*sa + sin(robot.attackBearing)*ca;
    ab_scale = 1/scale;
    quiver(x0, y0, xab*ab_scale,yab*ab_scale, 'k' );

    robotnames = {'Bot1','Bot2','Bot3','Bot4'};
    rolenames = {'Attack','Defend','Support','Goalie','Waiting'};
    colornames={'red','blue'};

    %robotID robotName robotRole 
    %BodyFSM HeadFSM
    %Team info
    %Voltage info

    str=sprintf('%s\n%s',...
	 robotnames{robot.id}, rolenames{robot.role});
    xtext=-1/scale;   xtext2=-0.4/scale;
   
    xt = xtext*ca + x0+xtext2;
    yt = xtext*sa + y0;

    b_name=text(xt, yt, str);
    set(b_name,'FontSize',8/scale);

  end

  function plot_ball(ball,scale)
    if (~isempty(ball))
      ball = [ball.x ball.y];
      %TODO: ball.t info
      xb = x0 + ball(1)*ca - ball(2)*sa;   
      yb = y0 + ball(1)*sa + ball(2)*ca;
      %hb = plot(xb, yb, [idColors(robot_struct.id) 'o']);
      hb = plot(xb, yb, 'ro');
      plot([x0 xb],[y0 yb],'r');
      set(hb, 'MarkerSize', 8/scale);
      %TODO: add last seen time info
    end
  end

  function plot_ball_mon(ball,scale)
    if ball.detect==1
      ball = [ball.x ball.y];
      %TODO: ball.t info
      xb = x0 + ball(1)*ca - ball(2)*sa;
      yb = y0 + ball(1)*sa + ball(2)*ca;
  %   hb = plot(xb, yb, [idColors(robot_struct.id) 'o']);
      hb = plot(xb, yb, 'ro');
      plot([x0 xb],[y0 yb],'r');
      set(hb, 'MarkerSize', 8/scale);
    end
  end
%}
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Monitor struct based plots (optional)
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  function plot_fov(fov)  %draw fov boundary
    x1 = fov.TL(1)*ca - fov.TL(2)*sa + x0;
    y1 = fov.TL(1)*sa + fov.TL(2)*ca + y0;
    x2 = fov.TR(1)*ca - fov.TR(2)*sa + x0;
    y2 = fov.TR(1)*sa + fov.TR(2)*ca + y0;
    plot([x1 x2],[y1 y2], 'k--');
    plot([x0 x1],[y0 y1], 'k--');
    plot([x0 x2],[y0 y2], 'k--');
  end
  
  function plot_goal(goal,scale)
    if( goal.detect==1 )
      if(goal.color==2) marker = 'm';% yellow
      else marker = 'b';end
      marker2 = strcat(marker,'+');
      if( goal.v1.scale ~= 0 )
        x1 = goal.v1.x*ca - goal.v1.y*sa + robot_struct.pose.x;
        y1 = goal.v1.x*sa + goal.v1.y*ca + robot_struct.pose.y;
        plot(x1,y1,marker2,'MarkerSize',12/scale);
        plot([x0 x1],[y0 y1],marker);
      end
      if( goal.v2.scale ~= 0 )
        x2 = goal.v2.x*ca - goal.v2.y*sa + robot_struct.pose.x;
        y2 = goal.v2.x*sa + goal.v2.y*ca + robot_struct.pose.y;
        plot(x2,y2,marker2,'MarkerSize',12/scale);
        plot([x0 x2],[y0 y2],marker);
      end
    end
  end

  function plot_landmark(landmark,scale)
    if (landmark.detect==1)
      if (landmark.color==2) marker='m';% yellow
      else marker='b';	end
      marker2 = strcat(marker,'x');
      x1 = landmark.v(1)*ca - landmark.v(2)*sa + robot_struct.pose.x;
      y1 = landmark.v(1)*sa + landmark.v(2)*ca + robot_struct.pose.y;
      plot(x1,y1,marker2,'MarkerSize',12/scale);
      plot([x0 x1],[y0 y1],marker);
    end
  end

end
