function dvmapInit
global DVMAP MAPS

if isempty(DVMAP) || ~isfield(DVMAP,'initialized') ||(DVMAP.initialized ~= 1)

  DVMAP.res        = MAPS.res;
  DVMAP.invRes     = MAPS.invRes;
  DVMAP.xmin       = MAPS.xmin;
  DVMAP.ymin       = MAPS.ymin;
  DVMAP.xmax       = MAPS.xmax;
  DVMAP.ymax       = MAPS.ymax;
  DVMAP.zmin       = MAPS.zmin;
  DVMAP.zmax       = MAPS.zmax;
  DVMAP.map.sizex  = MAPS.map.sizex;
  DVMAP.map.sizey  = MAPS.map.sizey;
  
  DVMAP.map.data   = zeros(DVMAP.map.sizex,DVMAP.map.sizey,'uint8');
  DVMAP.msgName    = '';
  
  DVMAP.name = 'Vertical Delta Map';
  DVMAP.initialized  = 1;
  disp('Obstacle map initialized');
end