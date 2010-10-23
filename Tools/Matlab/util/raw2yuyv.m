function yuyv = raw2yuyv(raw, width, height)
  % converts the raw data matrix to yuyv format
  yuyv_raw = typecast(raw(:), 'uint32');
  yuyv = reshape(yuyv_raw(1:width*height), [width, height]);

