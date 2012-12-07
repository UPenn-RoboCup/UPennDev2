function newMap = mapResize(oldMap,x,y)

newMap = oldMap;

if (x==0 && y==0)
  return;
end

dxi = ceil(abs(x)*oldMap.invRes);
dyi = ceil(abs(y)*oldMap.invRes);

if (x>0)
    xis  = (1+dxi):oldMap.map.sizex;
    nxis = 1:oldMap.map.sizex-dxi;
    length(xis);
    length(nxis);
elseif (x<0)
    xis  = 1:oldMap.map.sizex-dxi;
    nxis = (1+dxi):oldMap.map.sizex;
else
    xis  = 1:oldMap.map.sizex;
    nxis = 1:oldMap.map.sizex;
end

if (y>0)
    yis  = (1+dyi):oldMap.map.sizey;
    nyis = 1:oldMap.map.sizey -dyi;
elseif (y<0)
    yis  = 1:oldMap.map.sizey-dyi;
    nyis = (1+dyi):oldMap.map.sizey;
else
    yis  = 1:oldMap.map.sizey;
    nyis = 1:oldMap.map.sizey;
end

fprintf(1,'%d %d %d %d\n',xis(1),xis(end),nxis(1),nxis(end));
fprintf(1,'%d %d %d %d\n',yis(1),yis(end),nyis(1),nyis(end));

dataClass = class(oldMap.map.data);
keepMap = oldMap.map.data(xis,yis);
size(keepMap)

newMap.map.data = zeros(size(oldMap.map.data),dataClass);
newMap.map.data(nxis,nyis) = keepMap;
newMap.xmin = oldMap.xmin + x;
newMap.ymin = oldMap.ymin + y;
newMap.xmax = oldMap.xmax + x;
newMap.ymax = oldMap.ymax + y;


fprintf(1,'Map (%s) shifted. New bounding box: x(%f %f) y(%f %f)\n', ...
        newMap.name,newMap.xmin,newMap.xmax,newMap.ymin,newMap.ymax);
