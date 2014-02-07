function ret = posz_filter(posz,alpha,threshold)
  ret = posz;  
  for i=2:size(posz,1)-1
    for j=2:size(posz,2)-1
      cur = posz(i,j);

      leftcheck = abs(posz(i-1,j)-cur)<threshold;
      rightcheck = abs(posz(i+1,j)-cur)<threshold;
      topcheck = abs(posz(i,j-1)-cur)<threshold;
      bottomcheck = abs(posz(i,j+1)-cur)<threshold;

      lrcheck = abs(posz(i-1,j)-posz(i+1,j))<threshold;
      tbcheck = abs(posz(i-1,j)-posz(i+1,j))<threshold;

      sum = 0;
      count = 0;
      if leftcheck
	sum = sum + posz(i-1,j);
	count=count+1;
      end
      if rightcheck
	sum = sum + posz(i+1,j);
	count=count+1;
      end
      if topcheck
	sum = sum + posz(i,j-1);
	count=count+1;
      end
      if bottomcheck
	sum = sum + posz(i,j+1);
	count=count+1;
      end

      if count>0 
	ave = sum/count;
        ret(i,j) = alpha * ave + (1-alpha) * posz(i,j);
      else
        if lrcheck
	  sum = sum + posz(i-1,j) + posz(i+1,j);
	  count = count + 2;
	end
        if tbcheck
	  sum = sum + posz(i,j-1) + posz(i,j+1);
	  count = count + 2;
	end
	if count>0
	  ret(i,j) = sum/count; %replace that depth
	end
      end

    end
  end
end
