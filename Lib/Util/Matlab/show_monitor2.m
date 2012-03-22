function h=show_monitor( robots, scale, teamNumber, playerNumber )

  % Robot to display
  r_mon = robots{playerNumber,teamNumber}.get_monitor_struct();


  if( isempty(r_mon) )
    disp('Empty monitor struct!');
    return;
  end

  labelA = robots{playerNumber,teamNumber}.get_labelA();
  labelB = robots{playerNumber,teamNumber}.get_labelB();
  rgb = robots{playerNumber,teamNumber}.get_rgb();

  nTeams = size(robots,2);
  nPlayers = size(robots,1);

  % Colormap
  cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
  cmap=[cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];
 
  h1 = subplot(2,2,1);
  if( ~isempty(rgb) ) 
    plot_yuyv( h1, rgb );
    plot_overlay(r_mon,2);
  end

  h2 = subplot(2,2,2);
  if( ~isempty(labelA) ) 
    plot_label( h2, labelA, r_mon, 1, cmap);
    plot_overlay(r_mon,1);
  end

  h3 = subplot(2,2,3);
  plot_team( h3, r_mon)
      
  h4 = subplot(2,2,4);
  plot_surroundings( h4, r_mon );

  h.update = @update;

% Function details


  function plot_yuyv( handle, rgb )
    cla(handle);
    if( ~isempty(rgb) ) imagesc( rgb ); end
  end

  function plot_label( handle, label, r_mon, scale, cmap)
     % Process label
        cla(handle);
        if( ~isempty(label) )
            image(label);
            colormap(cmap);
            xlim([1 size(label,2)]);
            ylim([1 size(label,1)]);
	end
  end
  
  function plot_team( handle, r_mon)
     % Draw the field for localization reasons
    cla(handle);
    plot_field();
    r_struct = robots{playerNumber,teamNumber}.get_team_struct();
    plot_robot_monitor_struct( r_struct, r_mon,1 );
    hold off;
  end

end
