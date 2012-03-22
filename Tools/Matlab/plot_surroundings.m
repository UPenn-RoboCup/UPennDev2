function [ ] = plot_surroundings( handle, mon_struct )
    % NOTE: x and y are reversed because for the robot,
    % x is forward backward, but for plotting, y is up and down
    % Also, there is a negative one, since for the robot left is positive
    % TODO: check that this is right...
    
    cla( handle );
    % Assume that we can only see 3 meters left and right
    % Assume that we do not see objects very far behind us

    xlim([-2 2]);
    ylim([0 4]);
    hold on;

    %draw vision boundary
    fov=mon_struct.fov;
    plot(-[fov.TL(2) fov.TR(2)],[fov.TL(1) fov.TR(1)],'k--');
    plot(-[fov.BL(2) fov.BR(2)],[fov.BL(1) fov.BR(1)],'k--');
    plot(-[fov.TL(2) fov.BL(2)],[fov.TL(1) fov.BL(1)],'k--');
    plot(-[fov.TR(2) fov.BR(2)],[fov.TR(1) fov.BR(1)],'k--');
    
    ball = mon_struct.ball;
    if( ball.detect )
        plot(-1*ball.y, ball.x,'ro');
        strballpos = strcat('Ball: ',num2str(-1*ball.y,'%1.2f'),',',...
                            num2str(ball.x,'%1.2f'));
        b_name=text(-1*ball.y-0.3, ball.x+0.3, strballpos);
        set(b_name,'FontSize',8);
    end
    % TODO: Plot the right color
    goal = mon_struct.goal;
    if( goal.detect==1 )
        if(goal.color==2) % yellow
            marker = 'm';
        else
            marker = 'b';
        end
        marker = strcat(marker,'+');
        if( goal.v1.scale ~= 0 )
            plot(-1*goal.v1.y, goal.v1.x, marker,'MarkerSize',12);
	    g_name1=text(-1*goal.v1.y-0.30,goal.v1.x+0.3,sprintf('%.2f,%.2f',goal.v1.x,goal.v1.y));
	    set(g_name1,'FontSize',8);
        end
        if( goal.v2.scale ~= 0 )
            plot(-1*goal.v2.y, goal.v2.x, marker,'MarkerSize',12);
	    g_name2=text(-1*goal.v2.y-0.30,goal.v2.x+0.3,sprintf('%.2f,%.2f',goal.v2.x,goal.v2.y));
	    set(g_name2,'FontSize',8);
        end
    end

    landmark = mon_struct.landmark;
    if (landmark.detect==1)
	if (landmark.color==2) % yellow
	  marker1='m';marker2='b';
        else
	  marker1='b';marker2='m';
	end

        marker1 = strcat(marker1,'x');
        plot(-1*landmark.v(2), landmark.v(1), marker1,'MarkerSize',12);
        g_name2=text(-1*landmark.v(2)-0.30,landmark.v(1)+0.3,sprintf('%.2f,%.2f',landmark.v(1),landmark.v(2)));
        set(g_name2,'FontSize',8);

    end

%{
    bd = mon_struct.bd;
    if( bd.detect == 1 )
        % show top boundary
		plot(bd.topx,bd.topy);
		% show bottom boundary
		plot(bd.btmx,bd.btmy);
		% close boundary
		plot([bd.topx(1),bd.btmx(1)],[bd.topy(1),bd.btmy(1)]);
		plot([bd.topx(bd.nCol),bd.btmx(bd.nCol)],[bd.topy(bd.nCol),bd.btmy(bd.nCol)]);    
    end
%}
    hold off;
end

