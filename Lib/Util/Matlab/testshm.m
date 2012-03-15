image = shm('netImage');
format('short');
while (1) 
   raw = image.get_yuyv();
   yuyv = typecast(raw(:),'uint32');
   if (size(yuyv,1)>1)
       yuyv(1)
   end
end