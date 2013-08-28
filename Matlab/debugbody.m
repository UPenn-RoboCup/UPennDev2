function ret=debug()
  global DEBUGMON

  DEBUGMON=[];
  DEBUGMON.textstr={};
  DEBUGMON.textcount = 0;
  DEBUGMON.init=@init;
  DEBUGMON.addtext=@addtext;
  DEBUGMON.clearstr=@clearstr;
  num_line=5;

  ret = DEBUGMON;

  function init(h_text)
    DEBUGMON.h_text = h_text;
    DEBUGMON.textstr={};
    DEBUGMON.textcount = 0;
  end

  function addtext(str)
    if DEBUGMON.textcount<num_line
      DEBUGMON.textcount = DEBUGMON.textcount + 1;
      DEBUGMON.textstr{DEBUGMON.textcount}=str;
    else
      DEBUGMON.textstr = circshift(DEBUGMON.textstr,[0 -1]);
      DEBUGMON.textstr{DEBUGMON.textcount}=str;
    end
    set(DEBUGMON.h_text,'String',DEBUGMON.textstr);
  end

  function clearstr()
    DEBUGMON.textstr={};
    DEBUGMON.textcount = 0;
    set(DEBUGMON.h_text,'String',DEBUGMON.textstr);
  end
end

