function plot_overlay(r_mon,scale)
%This function plots overlaid vision information 
%Over the camera yuyv feed or labeled images

    if( ~isempty(r_mon) )
      if(r_mon.ball.detect==1)
        hold on;
        plot_ball( r_mon.ball, scale );
        hold off;
      end
      if( r_mon.goal.detect == 1 )
        hold on;
        if (~isempty(r_mon.goal.postStat1))
          plot_goalposts(r_mon.goal.postStat1,scale);
          if(r_mon.goal.type==3)
            plot_goalposts(r_mon.goal.postStat2,scale);
          end
        end
        hold off;
      end
      if( r_mon.landmark.detect == 1 )
        hold on;
	plot_landmark(r_mon.landmark,scale)
	hold off;
      end
    end


%Subfunctions


  function plot_ball( ballStats, scale )
    radius = (ballStats.axisMajor / 2) / scale;
    centroid = [ballStats.centroid.x ballStats.centroid.y] / scale;
    ballBox = [centroid(1)-radius centroid(2)-radius 2*radius 2*radius];
    plot( centroid(1), centroid(2),'k+')
    if( ~isnan(ballBox) )
      rectangle('Position', ballBox, 'Curvature',[1,1])
    end
  end


  function plot_goalposts( postStats, scale )

    x0=postStats.x;
    y0=postStats.y;
    w0=postStats.a/2;
    h0=postStats.b/2;
    a0=postStats.o;
    x0=x0/scale;y0=y0/scale;
    w0=w0/scale;h0=h0/scale;
    r=[cos(a0) sin(a0);-sin(a0) cos(a0)];
    x11=[x0 y0]+(r*[w0 h0]')';
    x12=[x0 y0]+(r*[-w0 h0]')';
    x21=[x0 y0]+(r*[w0 -h0]')';
    x22=[x0 y0]+(r*[-w0 -h0]')';

    goalcolor='r';goalwidth=2;

    plot([x11(1) x12(1)],[x11(2) x12(2)],goalcolor,'LineWidth',goalwidth);
    plot([x21(1) x22(1)],[x21(2) x22(2)],goalcolor,'LineWidth',goalwidth);
    plot([x12(1) x22(1)],[x12(2) x22(2)],goalcolor,'LineWidth',goalwidth);
    plot([x11(1) x21(1)],[x11(2) x21(2)],goalcolor,'LineWidth',goalwidth);

  end


  function plot_landmark(landmarkStats,scale)
    c1=landmarkStats.centroid1/scale;
    c2=landmarkStats.centroid2/scale;
    c3=landmarkStats.centroid3/scale;
    color=landmarkStats.color;


    m1=(c1+c2)/2; m2=(c2+c3)/2;
    c0 = c1-(m1-c1);c4=c3+(c3-m2);

    if color==2 % yellow 
	marker1='y';marker2='b';
    else
	marker1='b';marker2='y';
    end

    plot([c0(1) c4(1)],[c0(2) c4(2)],'g','LineWidth',8);
    plot([c0(1) m1(1)],[c0(2) m1(2)],marker1,'LineWidth',6);
    plot([m1(1) m2(1)],[m1(2) m2(2)],marker2,'LineWidth',6);
    plot([m2(1) c4(1)],[m2(2) c4(2)],marker1,'LineWidth',6);
  end
  
  function plot_freespace(free, scale)
    % TODO: Show freespace boundary in labelB
    hold on
    if (scale == 4)
      X = free.Bx;
      Y = free.By;
      plot(X,Y,'m--','LineWidth',2);
   else
      %X = free.Ax;
      %Y = free.Ay;
   end
   hold off;
  end

  function plot_horizon( horizon, scale )
    hold on;
    if (scale == 4)
      % labelB
      plot(horizon.hXB,horizon.hYB,'m--');
    else
      % labelA
      plot(horizon.hXA,horizon.hYA,'m--');
    end
    hold off;
  end

end

