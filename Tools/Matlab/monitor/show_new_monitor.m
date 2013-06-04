function h=show_new_monitor()
%This is new team monitor with logging and playback function

  global MONITOR LOGGER LUT;  
  global TEAM_LOG


  if isfield(TEAM_LOG,'count')==0
    TEAM_LOG=[];
    TEAM_LOG.count = 0;
    TEAM_LOG.viewcount = 1;
    TEAM_LOG.log_struct = {};
    TEAM_LOG.log_labelB = {};
    TEAM_LOG.is_logging = 0;
  end

  h.init=@init;
  h.update=@update;
  h.update_single=@update_single;
  h.update_team=@update_team;

  h.logging=0;
  h.lutname=0;
  h.is_webots=0;

  h.fieldtype=1; %0,1,2 for SPL/Kid/Teen
  h.count = 0; %To kill non-responding players from view
  h.timestamp=zeros(1,10);
  h.deadcount=zeros(1,10);


  function init(draw_team,target_fps)
    MONITOR.target_fps=target_fps;
    figure(1);
    clf;
    set(gcf,'position',[1 1 900 900]);
    MONITOR.enable10=2;  %Default 2
    MONITOR.hFpsText=uicontrol('Style','text',...
	'Units','Normalized', 'Position',[.40 0.93 0.20 0.04]);
    MONITOR.hButton1=uicontrol('Style','pushbutton','String','FPS -',...
	'Units','Normalized','Position',[.30 .93 .10 .04],'Callback',@button1);
    MONITOR.hButton2=uicontrol('Style','pushbutton','String','FPS +',...
	'Units','Normalized', 'Position',[.60 .93 .10 .04],'Callback',@button2);

    MONITOR.hButton3=uicontrol('Style','pushbutton','String','Kidsize',...
	'Units','Normalized', 'Position',[.02 .56 .07 .07],'Callback',@button3);


    MONITOR.hButton4=uicontrol('Style','pushbutton','String','Start',...
	'Units','Normalized', 'Position',[.25 .25 .10 .04],'Callback',@button4);
    MONITOR.hButton5=uicontrol('Style','pushbutton','String','Save',...
	'Units','Normalized', 'Position',[.35 .25 .10 .04],'Callback',@button5);

    MONITOR.hButton6=uicontrol('Style','pushbutton','String','<<',...
	'Units','Normalized', 'Position',[.50 .25 .10 .04],'Callback',@button6);
    MONITOR.hButton7=uicontrol('Style','pushbutton','String','<',...
	'Units','Normalized', 'Position',[.60 .25 .10 .04],'Callback',@button7);

    MONITOR.hButton8=uicontrol('Style','pushbutton','String','>',...
	'Units','Normalized', 'Position',[.80 .25 .10 .04],'Callback',@button8);
    MONITOR.hButton9=uicontrol('Style','pushbutton','String','>>',...
	'Units','Normalized', 'Position',[.90 .25 .10 .04],'Callback',@button9);

    MONITOR.hButton10=uicontrol('Style','pushbutton','String','',...
	'Units','Normalized', 'Position',[.70 .25 .10 .04]);




    for i=1:5
      MONITOR.infoTexts(i)=uicontrol('Style','text',...
	'Units','Normalized', 'Position',[0.16*(i-1)+0.12 0.71 0.145 0.08]);
      subplot(5,5,i);
      MONITOR.labelAxe(i) = gca;
    end
    for i=6:10
      MONITOR.infoTexts(i)=uicontrol('Style','text',...
      'Units','Normalized', 'Position',[0.16*(i-6)+0.12 0.01 0.145 0.08]);
      subplot(5,5,i+15);
      MONITOR.labelAxe(i) = gca;
    end
  end

  function update(robots,  teamNumber, playerNumber , draw_team, ignore_vision)
    if MONITOR.target_fps==0.5 %Paused state
       set(MONITOR.hFpsText,'String','Paused');
       pause(1);
    else
       tStart = tic;
       log_team_wireless(robots);
       draw_team_wireless(TEAM_LOG.viewcount);
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

  function log_team_wireless(robot_team)
    if (TEAM_LOG.is_logging ==1)||(TEAM_LOG.count==0)
      TEAM_LOG.count = TEAM_LOG.count + 1;
      MONITOR.count = MONITOR.count + 1;
    end
    if TEAM_LOG.is_logging==1 
      TEAM_LOG.viewcount = TEAM_LOG.count;
    end

    set(MONITOR.hButton10,'String', sprintf('%d/%d',...
	TEAM_LOG.viewcount, TEAM_LOG.count));



    TEAM_LOG.log_struct{TEAM_LOG.count} = {};
    TEAM_LOG.log_labelB{TEAM_LOG.count} = {};
    for i=1:10
      TEAM_LOG.log_struct{TEAM_LOG.count}{i}=[];
      TEAM_LOG.log_struct{TEAM_LOG.count}{i}.id = 0;
      r_struct = robot_team.get_team_struct_wireless(i);
      if r_struct.id>0
        timepassed = r_struct.time-MONITOR.timestamp(i);
        MONITOR.timestamp(i)=r_struct.time;
        if timepassed==0 MONITOR.deadcount(i) = MONITOR.deadcount(i)+1;
        else MONITOR.deadcount(i) = 0; 
        end
	deadcount_threshold = 50;
        if MONITOR.deadcount(i)<deadcount_threshold
          TEAM_LOG.log_struct{TEAM_LOG.count}{i}=r_struct;
          TEAM_LOG.log_labelB{TEAM_LOG.count}{i}=...
		robot_team.get_labelB_wireless(i);
        end
      end
    end
  end

  function draw_team_wireless(count)
    h_c=subplot(5,5,[6:20]);
    cla(h_c);
    hold on;
    plot_field(h_c,MONITOR.fieldtype);
    hold on;
    for i=1:10
      r_struct = TEAM_LOG.log_struct{count}{i};
      if r_struct.id>0
	labelB = TEAM_LOG.log_labelB{count}{i};
        h_c=subplot(5,5,[6:20]);
        plot_robot( r_struct, [],2,5,r_struct.robotName);
        updated = 0;
        axes(MONITOR.labelAxe(i));
        plot_label(labelB);
        plot_overlay_wireless(r_struct);
        [infostr textcolor]=robot_info(...
		r_struct,[],3,r_struct.robotName, r_struct.bodyState);
        set(MONITOR.infoTexts(i),'String',infostr);
      else
        axes(MONITOR.labelAxe(i));
        plot_label(zeros(60,80));
        set(MONITOR.infoTexts(i),'String','');
      end
    end
    hold off;
  end



  function button1(varargin)
    %0.5fps means paused state
    MONITOR.target_fps=max(0.5,MONITOR.target_fps/2);
  end
  function button2(varargin)
    MONITOR.target_fps=min(32,MONITOR.target_fps*2);
  end
  function button3(varargin)
    MONITOR.fieldtype=mod(MONITOR.fieldtype+1,3);
    if MONITOR.fieldtype==1 set(MONITOR.hButton3,'String', 'SPL');
    elseif MONITOR.fieldtype==2 set(MONITOR.hButton3,'String', 'TeenSize');
    else set(MONITOR.hButton3,'String', 'Kidsize');
    end
  end

  function button4(varargin)
    if TEAM_LOG.is_logging==0 
      TEAM_LOG.is_logging =1;
      set(MONITOR.hButton4,'String', 'Stop');
    else
      TEAM_LOG.is_logging =0;
      set(MONITOR.hButton4,'String', 'Start');
    end
  end
  function button5(varargin)
    filename = 'team_log.mat';
    save(filename, 'TEAM_LOG');
  end
  function button6(varargin)
    TEAM_LOG.is_logging = 0;
    TEAM_LOG.viewcount = max(1,TEAM_LOG.viewcount - 10);
    set(MONITOR.hButton4,'String', 'Start');
  end
  function button7(varargin)
    TEAM_LOG.is_logging = 0;
    TEAM_LOG.viewcount = max(1,TEAM_LOG.viewcount - 1);
    set(MONITOR.hButton4,'String', 'Start');
  end
  function button8(varargin)
    TEAM_LOG.is_logging = 0;
    TEAM_LOG.viewcount = min(TEAM_LOG.count,TEAM_LOG.viewcount + 1);
    set(MONITOR.hButton4,'String', 'Start');
  end
  function button9(varargin)
    TEAM_LOG.is_logging = 0;
    TEAM_LOG.viewcount = min(TEAM_LOG.count,TEAM_LOG.viewcount + 10);
    set(MONITOR.hButton4,'String', 'Start');
  end
end
