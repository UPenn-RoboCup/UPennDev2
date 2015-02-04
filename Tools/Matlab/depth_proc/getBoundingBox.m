function [ bbox ] = getBoundingBox( xind, yind )

[minyind,minyloc] = min(yind);
[maxyind,maxyloc] = max(yind);
[maxxind,maxxloc] = max(xind);
[minxind,minxloc] = min(xind);
bbox = unique([xind(minyloc) yind(minyloc); 
              xind(maxyloc) yind(maxyloc);   
              xind(minxloc) yind(minxloc);   
              xind(maxxloc) yind(maxxloc);   ],'rows');

end

