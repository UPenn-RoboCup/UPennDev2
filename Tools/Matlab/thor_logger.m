function h = thor_logger()
  % For thor, the compressed yuyv is sent over udp/zmq
  % and is djpeg'ed as rgb

  global LOGGER yuyvMontage;  
  h.init=@init;
  h.log_data=@log_data;
  h.save_log=@save_log;

  function init()
    % It's actually rgb...
    yuyvMontage=uint8([]);
    LOGGER.logging = 0;
    LOGGER.log_count=0;
  end

  %function log_data(yuyv,r_mon)
  function log_data(yuyv)
    LOGGER.log_count=LOGGER.log_count+1;
    yuyvMontage(:,:,:,LOGGER.log_count)=yuyv;
  end

  function save_log()
    %We still use the file name yuyv_xxxx
    %But now we store every log there as well
    if ~ exist('./logs','dir')
      mkdir('./logs');
    end
    savefile1 = ['./logs/yuyv_' datestr(now,30) '.mat'];
    fprintf('\nSaving yuyv file: %s...', savefile1)
    save(savefile1, 'yuyvMontage');
    init();
    disp('Done')
  end

end
  

