function ret = wheel_scanline_match(xpos)

	  i=1;
	  done = 0;
	  x_pre = 2.5;
	  connected_region_list= zeros(100,4);
	  is_connected = 0; 
	  connected_region_count = 0;

	  while i<=numel(xpos)
	    x = xpos(i);      
	    if x<0.7 && abs(x-x_pre)<0.05 % connected
          if is_connected==0
  	        is_connected = 1;
  	        connected_region_count = connected_region_count + 1;
	          connected_region_list(connected_region_count,1) = i;
	          connected_region_list(connected_region_count,2) = i;
	        else
		        connected_region_list(connected_region_count,2) = i;
          end
          connected_region_list(connected_region_count,3) = ...
	          connected_region_list(connected_region_count,3) +1;
          connected_region_list(connected_region_count,4) = ...
	          connected_region_list(connected_region_count,4) +x;
      else
        is_connected = 0;
      end
	    x_pre = x;
	    i=i+1;
	  end

%The list: min max count sum
    if connected_region_count>=2
      tot = connected_region_list(1:connected_region_count,4);
      count = connected_region_list(1:connected_region_count,3);
      ave = tot./count;
      [y,i] = sort(ave);
      if i(1)<i(2) 
       i_left = i(1);
       i_right = i(2);
       ave_left = y(1);
       ave_right = y(2);
      else
       i_left = i(2);
       i_right = i(1);
       ave_left = y(2);
       ave_right = y(1);
      end
      left_mid = (connected_region_list(i_left,1) + connected_region_list(i_left,2))/2;
      right_mid = (connected_region_list(i_right,1) + connected_region_list(i_right,2))/2;

      ret = [floor(left_mid+.5) floor(right_mid+.5)];
    elseif connected_region_count>0
      left_mid = (connected_region_list(1,1) + connected_region_list(1,2))/2;
      ret = [floor(left_mid+.5) floor(left_mid+.5)];
    else
      ret=[];
    end  
end
