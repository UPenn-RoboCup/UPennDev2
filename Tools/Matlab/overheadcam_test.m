a=shm_robot(1,1);

while 1
  raw = a.vcmImage.get_yuyv();
  width = 640;
  height = 360;
  yuyv = raw2yuyv(raw,width/2,height);
  [ycbcr,rgb] = yuyv2rgb(yuyv);
  imagesc(rgb);
  drawnow;
end
