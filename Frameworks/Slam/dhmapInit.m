function dhmapInit
global DHMAP MAPS

if isempty(DHMAP) || ~isfield(DHMAP,'initialized') ||(DHMAP.initialized ~= 1)

  DHMAP.res        = MAPS.res;
  DHMAP.invRes     = MAPS.invRes;
  DHMAP.xmin       = MAPS.xmin;
  DHMAP.ymin       = MAPS.ymin;
  DHMAP.xmax       = MAPS.xmax;
  DHMAP.ymax       = MAPS.ymax;
  DHMAP.zmin       = MAPS.zmin;
  DHMAP.zmax       = MAPS.zmax;
  DHMAP.map.sizex  = MAPS.map.sizex;
  DHMAP.map.sizey  = MAPS.map.sizey;
  
  DHMAP.map.data   = zeros(DHMAP.map.sizex,DHMAP.map.sizey,'uint8');
  DHMAP.msgName    = '';
  
  DHMAP.name = 'Horizontal Delta Map';
  DHMAP.initialized  = 1;
  disp('Obstacle map initialized');
end