function h=show_monitor_team( robots, scale, teamNumber, playerNumber , ignore_vision)

  % Robot to display

  %Draw common field 
  h_c=subplot(5,5,[1:15]);
  cla(h_c);
  plot_field();


  for i=1:length(playerNumber)
    r_struct = robots{playerNumber(i),teamNumber}.get_team_struct();

    r_mon = robots{playerNumber(i),teamNumber}.get_monitor_struct();
%   labelA = robots{playerNumber(i),teamNumber}.get_labelA();
    labelB = robots{playerNumber(i),teamNumber}.get_labelB();
%    rgb = robots{playerNumber(i),teamNumber}.get_rgb();

    h_c=subplot(5,5,[1:15]);
    plot_robot( r_struct, r_mon,2 );

    if ignore_vision==0 
      h1=subplot(5,5,15+playerNumber(i));
      plot_label( h1, labelB, r_mon, 1);
      plot_overlay(r_mon,4);

      h2=subplot(5,5,20+playerNumber(i));
      plot_surroundings( h2, r_mon );
    end

    h2=subplot(5,5,20+playerNumber(i));
    plot_info(r_struct,r_mon);
  end

  % subfunctions

  function plot_yuyv( handle, rgb )
    if( ~isempty(rgb) ) 
      cla(handle);
      imagesc( rgb ); 
    end
  end

  function plot_label( handle, label, r_mon, scale)
    % Colormap
    cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
    cmap=[cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];

    if( ~isempty(label) )
      cla(handle); 
      image(label);
      colormap(cmap);
      xlim([1 size(label,2)]);
      ylim([1 size(label,1)]);
    end
  end
  
  function plot_info(robot,r_mon)

    robotnames = {'Bot1','Bot2','Bot3','Bot4'};
    rolenames = {'Attacker','Defender','Supporter','Goalie','Waiting'};
    colornames={'red','blue'};

    %robotID robotName robotRole 
    %BodyFSM HeadFSM
    %Team info
    %Voltage info

    str=sprintf('#%d %s  %s\n%s %s\n %.1fV',...
	 robot.id, robotnames{robot.id}, rolenames{robot.role},...
         char(r_mon.fsm.head),   char(r_mon.fsm.body), robot.battery_level);


%  char(r_mon.fsm.game)
%  char(r_mon.fsm.motion)

%{
        r.teamNumber = h.gcmTeam.get_number();
        r.teamColor = h.gcmTeam.get_color();
        r.id = h.gcmTeam.get_player_id();
        r.role = h.gcmTeam.get_role();

    str=sprintf('#%d %s  %s\n %s %s\n Teammate: %s\n %.1fV %dC \nTM$
                   i,robotName, roleNames{data(i).role}, data(i).HeadFSM, data($
                    data(i).teammate,data(i).battery/10, data(i).temp,tAgo,data$
                h_xlabel=xlabel(str);
%}
    h_xlabel=xlabel(str);

    set(h_xlabel,'Color','k');

  end

end
