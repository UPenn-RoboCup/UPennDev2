function h = plot_robot_monitor_struct(robot_struct,r_mon,scale,drawlevel)
% This function shows the robot over the field map
% Level 1: show position only
% Level 2: show position and vision info
% Level 3: show position, vision info and fov info
% Level 4: show position, vision info and particles

  x0 = robot_struct.pose.x;
  y0 = robot_struct.pose.y;
  ca = cos(robot_struct.pose.a);
  sa = sin(robot_struct.pose.a);
      
  hold on;

  if robot_struct.fall
    ca=1;sa=0;
    plot_fallen_robot(robot_struct,scale)
    plot_info(robot_struct,scale);
  else
    plot_robot(robot_struct,scale);
    plot_info(robot_struct,scale);
    plot_ball(robot_struct,scale);

    if drawlevel>1
      plot_goal(r_mon.goal,scale);
      plot_landmark(r_mon.landmark,scale);
      if drawlevel==3
        plot_fov(r_mon.fov);
      elseif drawlevel==4
	plot_particle(r_mon.particle);
      end
    end
  end

  hold off;

%subfunctions

  function plot_fallen_robot(robot,scale)
    xr = x0+[-0.10  0  .10  0]*2/scale;
    yr = y0+[0    .10   0 -.10]*2/scale;

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
  end


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
  end

  function plot_info(robot,angle)
    robotnames = {'Bot1','Bot2','Bot3','Bot4','Bot5','Bot6'};
    rolenames = {'Unknown','Attack','Defend','Support','Goalie','Waiting'};
    colornames={'red','blue'};

    %robotID robotName robotRole 
    %BodyFSM HeadFSM
    %Team info
    %Voltage info

    str=sprintf('%s\n%s',...
	 robotnames{robot.id}, rolenames{robot.role+1});
    xtext=-1/scale;   xtext2=-0.4/scale;
   
    xt = xtext*ca + x0+xtext2;
    yt = xtext*sa + y0;

    b_name=text(xt, yt, str);
    set(b_name,'FontSize',8/scale);
  end

  function plot_ball(robot,scale)
    if (~isempty(robot.ball))
      ball = [robot.ball.x robot.ball.y robot.time-robot.ball.t];
      ballt=ball(3);
      xb = x0 + ball(1)*ca - ball(2)*sa;   
      yb = y0 + ball(1)*sa + ball(2)*ca;
      %hb = plot(xb, yb, [idColors(robot_struct.id) 'o']);
      hb = plot(xb, yb, 'ro');
 
      ball_vel=[robot.ball.vx robot.ball.vy];
      xbv = xb + ball_vel(1)*ca - ball_vel(2)*sa;   
      ybv = yb + ball_vel(1)*sa + ball_vel(2)*ca;

      ab_scale = 1/scale;
%      quiver(xb, yb, xab*ab_scale,yab*ab_scale, 'k' );
      plot([xb xbv],[yb ybv],'r--','LineWidth',2/scale);

      if ballt<0.5 
        plot([x0 xb],[y0 yb],'r');
        set(hb, 'MarkerSize', 8/scale);
      else
        %TODO: add last seen time info
      end
    end
  end

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
      marker2 = strcat(marker,'--');
      if( goal.v1.scale ~= 0 )

        if goal.type==0 
          marker1 = strcat(marker,'+');%Unknown post
	elseif goal.type==2
          marker1 = strcat(marker,'>');%Right post
	else
          marker1 = strcat(marker,'<');%Left or two post
	end
        x1 = goal.v1.x*ca - goal.v1.y*sa + robot_struct.pose.x;
        y1 = goal.v1.x*sa + goal.v1.y*ca + robot_struct.pose.y;
        plot(x1,y1,marker1,'MarkerSize',12/scale);
        plot([x0 x1],[y0 y1],marker2);
      end
      if( goal.v2.scale ~= 0 )
        marker1 = strcat(marker,'>');%Left post
        x2 = goal.v2.x*ca - goal.v2.y*sa + robot_struct.pose.x;
        y2 = goal.v2.x*sa + goal.v2.y*ca + robot_struct.pose.y;
        plot(x2,y2,marker1,'MarkerSize',12/scale);
        plot([x0 x2],[y0 y2],marker2);
      end
    end
  end

  function plot_landmark(landmark,scale)
    if (landmark.detect==1)
      if (landmark.color==2) marker='m';% yellow
      else marker='b';	end
      marker2 = strcat(marker,'x');
      marker3 = strcat(marker,'--');
      x1 = landmark.v(1)*ca - landmark.v(2)*sa + robot_struct.pose.x;
      y1 = landmark.v(1)*sa + landmark.v(2)*ca + robot_struct.pose.y;
      plot(x1,y1,marker2,'MarkerSize',12/scale);
      plot([x0 x1],[y0 y1],marker3);
    end
  end

  function plot_particle(particle)
    plot(particle.x,particle.y,'x')
  end

end
