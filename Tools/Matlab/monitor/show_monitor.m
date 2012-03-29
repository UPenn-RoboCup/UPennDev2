function h=show_monitor()
  global MONITOR LOGGER LUT;  
  h.init=@init;
  h.update=@update;
  h.update_single=@update_single;
  h.update_team=@update_team;

  %Five screen for single monitor
  h.enable1=1;
  h.enable2=1;
  h.enable3=1;
  h.enable4=1;
  h.enable5=1;

  %two subscreen for team monitor
  h.enable8=1;   %Label mode, 1/2/0
  h.enable9=1;   %Label mode, 1/0
  h.enable10=1;  %Map mode, 1/2/3

  h.logging=0;
  h.lutname=0;

  % subfunctions
  function init(draw_team,target_fps)
    MONITOR.target_fps=target_fps;
    figure(1);
    clf;
    if draw_team>0 
      set(gcf,'position',[1 1 900 900]);
      MONITOR.hFpsText=uicontrol('Style','text','Position',[380 870 200 20]);

      MONITOR.hButton6=uicontrol('Style','pushbutton','String','FPS -',...
	'Position',[300 870 70 20],'Callback',@button6);

      MONITOR.hButton7=uicontrol('Style','pushbutton','String','FPS +',...
	'Position',[600 870 70 20],'Callback',@button7);

      MONITOR.hButton8=uicontrol('Style','pushbutton','String','LABEL',...
	'Position',[20 260 70 40],'Callback',@button8);

      MONITOR.hButton9=uicontrol('Style','pushbutton','String','2D',...
	'Position',[20 200 70 40],'Callback',@button9);

      MONITOR.hButton10=uicontrol('Style','pushbutton','String','MAP1',...
	'Position',[20 600 70 40],'Callback',@button10);

    else
      LOGGER=logger();
      LOGGER.init();
      set(gcf,'Position',[1 1 1000 600])
      MONITOR.hFpsText=uicontrol('Style','text','Position',[380 570 200 20]);

      MONITOR.hDebugText=uicontrol('Style','text','Position',[770 60 200 500]);

      MONITOR.hButton1=uicontrol('Style','pushbutton','String','YUYV ON',...
	'Position',[20 500 70 40],'Callback',@button1);

      MONITOR.hButton2=uicontrol('Style','pushbutton','String','LABEL A',...
	'Position',[20 440 70 40],'Callback',@button2);

      MONITOR.hButton3=uicontrol('Style','pushbutton','String','MAP A',...
	'Position',[20 380 70 40],'Callback',@button3);

      MONITOR.hButton4=uicontrol('Style','pushbutton','String','2D ON',...
	'Position',[20 320 70 40],'Callback',@button4);

      MONITOR.hButton5=uicontrol('Style','pushbutton','String','DEBUG ON',...
	'Position',[20 260 70 40],'Callback',@button5);

      MONITOR.hButton6=uicontrol('Style','pushbutton','String','FPS -',...
	'Position',[300 570 70 20],'Callback',@button6);

      MONITOR.hButton7=uicontrol('Style','pushbutton','String','FPS +',...
	'Position',[600 570 70 20],'Callback',@button7);

      MONITOR.hButton11=uicontrol('Style','pushbutton','String','LOG',...
	'Position',[20 200 70 40],'Callback',@button11);

      MONITOR.hButton12=uicontrol('Style','pushbutton','String','Load LUT',...
	'Position',[700 570 250 20],'Callback',@button12);

    end
  end

  function update(robots,  teamNumber, playerNumber , draw_team, ignore_vision)
    if MONITOR.target_fps==0.5 %Paused state
       set(MONITOR.hFpsText,'String','Paused');
       pause(1);
    else 
       tStart = tic;
       if draw_team>0 
         update_team(robots, teamNumber, playerNumber , ignore_vision)
       else
         update_single( robots, teamNumber, playerNumber )
       end
       drawnow;
       tElapsed=toc(tStart);

       set(MONITOR.hFpsText,'String',...
  	 sprintf('Plot: %d ms FPS: %.1f / %.1f',	floor(tElapsed*1000),...
         min(1/tElapsed,MONITOR.target_fps), MONITOR.target_fps ));
       if(tElapsed<1/MONITOR.target_fps)
         pause( 1/MONITOR.target_fps-tElapsed );
       end
    end
  end

  function update_single( robots, teamNumber, playerNumber )
    % Robot to display
    r_struct = robots{playerNumber,teamNumber}.get_team_struct();
    r_mon = robots{playerNumber,teamNumber}.get_monitor_struct();

is_webots=1;%TODO
is_webots=0;%TODO

    if( isempty(r_mon) )
      disp('Empty monitor struct!'); return;
    end

    if MONITOR.enable1
      if MONITOR.enable1==1
        MONITOR.h1 = subplot(4,5,[1 2 6 7]);
        yuyv = robots{playerNumber,teamNumber}.get_yuyv();
        [ycbcr,rgb]=yuyv2rgb(yuyv);
        plot_rgb( MONITOR.h1, rgb );
      elseif MONITOR.enable1==2
        MONITOR.h1 = subplot(4,5,[1 2 6 7]);
        yuyv2 = robots{playerNumber,teamNumber}.get_yuyv2();
        [ycbcr,rgb]=yuyv2rgb(yuyv2);
        plot_rgb( MONITOR.h1, rgb );
      end

      %webots use non-subsampled label (2x size of yuyv)
      if is_webots
        plot_overlay(r_mon,2*MONITOR.enable1);
      else
        plot_overlay(r_mon,1*MONITOR.enable1);
      end
    end

    if MONITOR.enable2==1
      MONITOR.h2 = subplot(4,5,[3 4 8 9]);
      labelA = robots{playerNumber,teamNumber}.get_labelA();
      plot_label( MONITOR.h2, labelA, r_mon, 1);
      plot_overlay(r_mon,1);
    elseif MONITOR.enable2==2
      MONITOR.h2 = subplot(4,5,[3 4 8 9]);
      labelB = robots{playerNumber,teamNumber}.get_labelB();
      plot_label( MONITOR.h2, labelB, r_mon, 4);
      plot_overlay(r_mon,4);
    elseif (MONITOR.enable2==3) && (~isempty(MONITOR.lutname))
      yuyv = robots{playerNumber,teamNumber}.get_yuyv();
      yuv=yuyv2yuv(yuyv);
      MONITOR.h2 = subplot(4,5,[3 4 8 9]);
      plot_label_lut(MONITOR.h2,yuv)
    end

    if MONITOR.enable3
      MONITOR.h3 = subplot(4,5,[11 12 16 17]);
      plot_field(MONITOR.h3);
      plot_robot( r_struct, r_mon,1,MONITOR.enable3 );
    end

    if MONITOR.enable4
      MONITOR.h4 = subplot(4,5,[13 14 18 19]);
      plot_surroundings( MONITOR.h4, r_mon );
    end
    
    if MONITOR.enable5
      set(MONITOR.hDebugText,'String',r_mon.debug.message);
    end

    if MONITOR.logging
      LOGGER.log_yuyv(yuyv + 0);
      logstr=sprintf('%d/100',LOGGER.log_count);
      set(MONITOR.hButton11,'String', logstr);
      if LOGGER.log_count==100 
        LOGGER.save_log();
      end
    end

  end

  function update_team(robots, teamNumber, playerNumber , ignore_vision)

    %Draw common field 
    h_c=subplot(5,5,[1:15]);
    plot_field(h_c);

    for i=1:length(playerNumber)
      r_struct = robots{playerNumber(i),teamNumber}.get_team_struct();
      r_mon = robots{playerNumber(i),teamNumber}.get_monitor_struct();
      %labelA = robots{playerNumber(i),teamNumber}.get_labelA();
      labelB = robots{playerNumber(i),teamNumber}.get_labelB();
      %rgb = robots{playerNumber(i),teamNumber}.get_rgb();

      h_c=subplot(5,5,[1:15]);
      plot_robot( r_struct, r_mon,2,MONITOR.enable10);

      if MONITOR.enable8==1 && ignore_vision==0
        h1=subplot(5,5,15+playerNumber(i));
        plot_label( h1, labelB, r_mon, 1);
        plot_overlay(r_mon,4);
      elseif MONITOR.enable8==2
        h1=subplot(5,5,15+playerNumber(i));
        cla(h1);
        plot_overlay(r_mon,4);
      end
	
      if MONITOR.enable9
        h2=subplot(5,5,20+playerNumber(i));
        plot_surroundings( h2, r_mon );
      end

      h2=subplot(5,5,20+playerNumber(i));
      plot_info(r_struct,r_mon);
    end
  end


  function plot_rgb( handle, rgb )
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

  function plot_label_lut(handle,yuv)

    siz=size(yuv);
    index = yuv2index(yuv, [64 64 64]);
    im_display=[];
    label=LUT(index)+1;
    cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
    cmap=[cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];
    cla(handle);
    image(label);
    colormap(cmap);
  end

  function plot_info(robot,r_mon)
    robotnames = {'Bot1','Bot2','Bot3','Bot4'};
    rolenames = {'','Attacker','Defender','Supporter','Goalie','Waiting'};
    colornames={'red','blue'};

    str=sprintf('#%d %s  %s\n%s %s\n %.1fV',...
	 robot.id, robotnames{robot.id}, rolenames{robot.role+1},...
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

  function button1(varargin)
    MONITOR.enable1=mod(MONITOR.enable1+1,3);
    if MONITOR.enable1==1 set(MONITOR.hButton1,'String', 'YUYV1');
    elseif MONITOR.enable1==2 set(MONITOR.hButton1,'String', 'YUYV2');
    else set(MONITOR.hButton1,'String', 'YUYV OFF');
      cla(MONITOR.h1);
    end
  end

  function button2(varargin)
    MONITOR.enable2=mod(MONITOR.enable2+1,4);
    if MONITOR.lutname==0 
      MONITOR.enable2=mod(MONITOR.enable2,3);
    end 
    if MONITOR.enable2==1 set(MONITOR.hButton2,'String', 'LABEL A');
    elseif MONITOR.enable2==2 set(MONITOR.hButton2,'String', 'LABEL B');
    elseif MONITOR.enable2==3 set(MONITOR.hButton2,'String', 'LUT');
    else set(MONITOR.hButton2,'String', 'LABEL OFF');
      cla(MONITOR.h2);
    end
  end

  function button3(varargin)
    MONITOR.enable3=mod(MONITOR.enable3+1,4);
    if MONITOR.enable3==1 set(MONITOR.hButton3,'String', 'MAP1');
    elseif MONITOR.enable3==2 set(MONITOR.hButton3,'String', 'MAP2');
    elseif MONITOR.enable3==3 set(MONITOR.hButton3,'String', 'MAP3');
    else set(MONITOR.hButton3,'String', 'MAP OFF');
      cla(MONITOR.h3);
    end
  end

  function button4(varargin)
    MONITOR.enable4=1-MONITOR.enable4;
    if MONITOR.enable4 set(MONITOR.hButton4,'String', '2D ON');
    else set(MONITOR.hButton4,'String', '2D OFF');
      cla(MONITOR.h4);
    end
  end

  function button5(varargin)
    MONITOR.enable5=1-MONITOR.enable5;
    if MONITOR.enable5 set(MONITOR.hButton5,'String', 'DEBUG ON');
    else set(MONITOR.hButton5,'String', 'DEBUG OFF');
      set(MONITOR.hDebugText,'String','');
    end
  end

  function button6(varargin)
    %0.5fps means paused state
    MONITOR.target_fps=max(0.5,MONITOR.target_fps/2);
  end

  function button7(varargin)
    MONITOR.target_fps=min(32,MONITOR.target_fps*2);
  end

  function button8(varargin)
    MONITOR.enable8=mod(MONITOR.enable8+1,3);
    if MONITOR.enable8==1 set(MONITOR.hButton8,'String', 'LABEL');
    elseif MONITOR.enable8==2 set(MONITOR.hButton8,'String', 'Overlay');
    else set(MONITOR.hButton8,'String', 'OFF');
    end
  end

  function button9(varargin)
    MONITOR.enable9=1-MONITOR.enable9;
    if MONITOR.enable9 set(MONITOR.hButton9,'String', '2D ON');
    else set(MONITOR.hButton9,'String', '2D OFF');
    end
  end

  function button10(varargin)
    MONITOR.enable10=mod(MONITOR.enable10,3)+1;  %1,2,3
    if MONITOR.enable10==1 set(MONITOR.hButton10,'String', 'MAP1');
    elseif MONITOR.enable10==2 set(MONITOR.hButton10,'String', 'MAP2');
    elseif MONITOR.enable10==3 set(MONITOR.hButton10,'String', 'MAP3');
    end
  end

  function button11(varargin)
    MONITOR.logging=1-MONITOR.logging;
  end

  function button12(varargin) % Load lut file
    [filename, pathname] = uigetfile('*.raw', 'Select lut file to load');
    if (filename ~= 0)
      MONITOR.lutname=filename;
      fid = fopen([pathname filename], 'r');
      LUT = fread(fid, 'uint8');
      fclose(fid);
      set(MONITOR.hButton12,'String', filename);
    end
  end
end
