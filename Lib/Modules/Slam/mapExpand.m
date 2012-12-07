function newMap = mapExpand(oldMap,x,y)

newMap = oldMap;

if (x==0 && y==0)
  return;
end

%{
new map will look like this

y
^
|m13 m23 m33
|m12 OLD m32
|m11 m21 m31
|---------->x
%}

m21=[];
m12=[];

m32=[];
m23=[];

dataType = class(oldMap.map.data);

if (x<0)
  m12 = zeros(ceil(-x*oldMap.invRes),oldMap.map.sizey,dataType);
elseif (x>0)
  m32 = zeros(ceil(x*oldMap.invRes),oldMap.map.sizey,dataType);
end

if (y<0)
  m21 = zeros(oldMap.map.sizex,ceil(-y*oldMap.invRes),dataType);
elseif (y>0)
  m23 = zeros(oldMap.map.sizex,ceil(y*oldMap.invRes),dataType);
end

m11 = zeros([size(m12,1) size(m21,2)],dataType);
m31 = zeros([size(m32,1) size(m21,2)],dataType);
m13 = zeros([size(m12,1) size(m23,2)],dataType);
m33 = zeros([size(m32,1) size(m23,2)],dataType);

if (numel(m11)==0), m11=[]; end
if (numel(m31)==0), m31=[]; end
if (numel(m13)==0), m13=[]; end
if (numel(m33)==0), m33=[]; end

newMap.map.data = [m11     m12         m13;
                   m21 oldMap.map.data m23;
                   m31     m32         m33];
                 
                 
newMap.xmin       = oldMap.xmin - size(m12,1)*oldMap.res;
newMap.ymin       = oldMap.ymin - size(m21,2)*oldMap.res;
newMap.xmax       = oldMap.xmax + size(m32,1)*oldMap.res;
newMap.ymax       = oldMap.ymax + size(m23,2)*oldMap.res;

newMap.map.sizex  = (newMap.xmax - newMap.xmin) * newMap.invRes + 1;   %do we need to round or ceil here????
newMap.map.sizey  = (newMap.ymax - newMap.ymin) * newMap.invRes + 1;

fprintf(1,'Map (%s) resized. New dimensions: x(%f %f) y(%f %f)\n', ...
        newMap.name,newMap.xmin,newMap.xmax,newMap.ymin,newMap.ymax);
