function h=show_saffir_monitor()
  global MONITOR LOGGER LUT;  

  h.init=@init;
  h.update=@update;
 

 % subfunctions
  function init()
    MONITOR.target_fps=target_fps;
    figure(1);
    clf;

      set(gcf,'Position',[1 1 1000 600])
      MONITOR.hFpsText=uicontrol('Style','text',...
	'Units','Normalized', 'Position',[.40 0.93 0.20 0.04]);

      MONITOR.hButton6=uicontrol('Style','pushbutton','String','FPS -',...
	'Units','Normalized','Position',[.30 .93 .10 .04],'Callback',@button6);

      MONITOR.hButton7=uicontrol('Style','pushbutton','String','FPS +',...
	'Units','Normalized', 'Position',[.60 .93 .10 .04],'Callback',@button7);

  end

  function update()
    if MONITOR.target_fps==0.5 %Paused state
       set(MONITOR.hFpsText,'String','Paused');
       pause(1);
    else 
       tStart = tic;

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

  function plot_grid(map)
    %Update occupancy map     

    %occupancy map: -1 to 1
    siz=sqrt(length(map)/6/4);    
    map=reshape(map,[4*siz 6*siz]);
    map_black=max(0,map);
    map_green=max(0,-map);

    rgbc=zeros([4*siz 6*siz 3]);
    rgbc(:,:,1)=1-map_black-map_green;
    rgbc(:,:,2)=1-map_black;
    rgbc(:,:,3)=1-map_black-map_green;
    image('XData',[-3:1/siz:3],'YData',[-2:1/siz:2],...
	'CData',rgbc);  
  end

  function button6(varargin)
    %0.5fps means paused state
    MONITOR.target_fps=max(0.5,MONITOR.target_fps/2);
  end

  function button7(varargin)
    MONITOR.target_fps=min(32,MONITOR.target_fps*2);
  end

end
