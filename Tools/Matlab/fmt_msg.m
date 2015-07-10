function [y] = fmt_msg(metadata, raw)
    msg_id = char(metadata.id);
    if strcmp(msg_id,'world')
      count_world = count_world + 1;
      data_world.meta = metadata;
      data_world.raw = raw;
      data_world.recv = true;
    end
    if strcmp(msg_id,'detect')
      count_detect = count_detect + 1;
      data_detect.meta = metadata;
      data_detect.raw = raw;
      data_detect.recv = true;
    end
    if strcmp(msg_id,'labelA')
      count_labelA = count_labelA + 1;
      data_labelA.meta = metadata;
      data_labelA.raw = raw;
      data_labelA.recv = true;
    end
    if strcmp(msg_id,'head_camera')
      count_cam = count_cam + 1;
      data_yuyv.meta = metadata;
      data_yuyv.raw = raw;
      data_yuyv.recv = true;
    end 
end