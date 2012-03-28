function h=logger()
  %SJ: generate two separate files, yuyvMontage and LOG
  %Now we directly generate yuyv files, so no need for convert

  global LOGGER LOG yuyvMontage;  
  h.init=@init;
  h.log_yuyv=@log_yuyv;
  h.save_log=@save_log;

  function init()
    yuyvMontage=uint32([]);
    LOG=[];
    LOGGER.logging = 0;
    LOGGER.log_count=0;
  end

  function log_yuyv(yuyv)
    LOGGER.log_count=LOGGER.log_count+1;
    yuyvMontage(:,:,1,LOGGER.log_count)=yuyv;

%TODO: log other data
  end

  function save_log()
    %Save yuyvMontage 
    savefile1 = ['./colortable/yuyv_' datestr(now,30) '.mat'];
    fprintf('\nSaving yuyv file: %s...', savefile1)
    save(savefile1, 'yuyvMontage');
%{
    %Save LOG  
    savefile2 = ['./colortable/log_' datestr(now,30) '.mat'];
    fprintf('\nSaving Log file: %s...', savefile2)
    save(savefile2, 'LOG');
%}
    init();
  end

end
  

