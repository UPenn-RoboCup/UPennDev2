
% (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
% ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
% University of Pennsylvania

function [xStart yStart thStart] = FindStartPose(xStart,yStart,thStart, pxs,pys)
global REF_MAP OMAP


REF_MAP

if (~isempty(REF_MAP))
  
 fprintf('using initial search position : %f %f %f\n',xStart,yStart,thStart);
  
  dyaw = rad(0.2);
  dx   = 0.02;
  dy   = 0.02;

  nyaw = 161;
  nxs  = 41;
  nys  = 41;

  yawRange = floor(nyaw/2);
  xRange   = floor(nxs/2); 
  yRange   = floor(nys/2);

  %create the candidate locations in each dimension
  aCand = (-yawRange:yawRange)*dyaw + thStart;
  xCand = (-xRange:xRange)*dx + xStart;
  yCand = (-yRange:yRange)*dy + yStart;

  tic
  ScanMatch2D('setBoundaries',REF_MAP.xmin,REF_MAP.ymin,REF_MAP.xmax,REF_MAP.ymax);
  ScanMatch2D('setResolution',REF_MAP.res);
  
  %get a local 3D sampling of pose likelihood
  hits = ScanMatch2D('match',REF_MAP.map,pxs,pys, ...
                  xCand,yCand,aCand);
                
  toc

  %find maximum
  [hmax imax] = max(hits(:));
  [kmax mmax jmax] = ind2sub([nxs,nys,nyaw],imax);

  x  = xCand(kmax);
  y  = yCand(mmax);
  th = aCand(jmax);

  fprintf('initial values with maximum = %f \n',hmax);
  xStart  = x
  yStart  = y
  thStart = th  
end

ScanMatch2D('setBoundaries',OMAP.xmin,OMAP.ymin,OMAP.xmax,OMAP.ymax);
ScanMatch2D('setResolution',OMAP.res);

