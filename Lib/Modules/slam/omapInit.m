function omapInit
global OMAP MAPS

if isempty(OMAP) || ~isfield(OMAP,'initialized') ||(OMAP.initialized ~= 1)

  OMAP.res        = MAPS.res;
  OMAP.invRes     = MAPS.invRes;
  OMAP.xmin       = MAPS.xmin;
  OMAP.ymin       = MAPS.ymin;
  OMAP.xmax       = MAPS.xmax;
  OMAP.ymax       = MAPS.ymax;
  OMAP.zmin       = MAPS.zmin;
  OMAP.zmax       = MAPS.zmax;
  OMAP.map.sizex  = MAPS.map.sizex;
  OMAP.map.sizey  = MAPS.map.sizey;
  
  OMAP.map.data   = zeros(OMAP.map.sizex,OMAP.map.sizey,'uint8');
  OMAP.msgName    = [GetRobotName '/ObstacleMap2D_map2d'];
  
  OMAP.name = 'Occupancy Map';
  
  ipcInit;
  if checkVis, ipcAPIDefine(OMAP.msgName,VisMap2DSerializer('getFormat')); end
  OMAP.initialized  = 1;
  disp('Obstacle map initialized');
end
