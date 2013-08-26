function ret=netmonbody()
  global NETMON

  NETMON=[];
  NETMON.n_history = 150;
  NETMON.max_y = 100*1024;
  NETMON.num_callbacks = 7;
  NETMON.recent_usage = zeros(NETMON.n_history,NETMON.num_callbacks);
  NETMON.total_usage = zeros(NETMON.num_callbacks,1,'uint32');
  NETMON.t_last_usage = zeros(NETMON.num_callbacks,1,'uint32');

  NETMON.init = @init;
  NETMON.update = @update;
  NETMON.get_total_recent_usage = @get_total_recent_usage;
  NETMON.redraw = @redraw;

  NETMON.tLastDraw=0;

  ret = NETMON;

  function init(a,h_text)
    NETMON.a = a;
    NETMON.h_text = h_text;
    for i=1:numel(NETMON.t_last_usage)
      NETMON.t_last_usage(i) = tic;
    end
    NETMON.line = plot(a, NETMON.recent_usage / 1024);
    set(a,'XLim',[0 NETMON.n_history]);
    set(a,'YLim',[0 NETMON.max_y]);
    set(a,'YGrid','on');
    NETMON.tLastDraw=tic;
  end

  function update(callback_no, nBytes)
    NETMON.t_last_usage(callback_no) = tic;
    NETMON.total_usage(callback_no) = ...
			NETMON.total_usage(callback_no) + nBytes; 
    NETMON.recent_usage(1,callback_no) = ...
			NETMON.recent_usage(1,callback_no) +nBytes; 
  end

  function ret=get_total_recent_usage()
    ret = sum(NETMON.recent_usage(1,:));
  end

  function redraw(fps,battery)
    t_diff = toc(NETMON.tLastDraw);
    rate = NETMON.get_total_recent_usage() / t_diff/1024;
	  set( NETMON.line, {'YDATA'}, num2cell( NETMON.recent_usage, 1 )' );
    set( NETMON.h_text, 'string', ...
			sprintf('%2.0f FPS | %4.0f kB/s  %.2f V\n',...
      fps, rate, battery ));
    NETMON.recent_usage = circshift(NETMON.recent_usage,1);
    NETMON.recent_usage(1,:) = 0;
    NETMON.tLastDraw = tic;
  end

end

