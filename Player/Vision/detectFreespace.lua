module(..., package.seeall);

require('Config');	-- For Ball and Goal Size
require('ImageProc');
require('HeadTransform');	-- For Projection
require('Vision');
require('Debug');
require('shm');
require('vcm');
require('Body');
require('vector');

obstSize = 0.2;
-- Define Color
colorOrange = Config.color.orange;
colorYellow = Config.color.yellow;
colorCyan = Config.color.cyan;
colorField = Config.color.field;
colorWhite = Config.color.white;

use_point_goal=Config.vision.use_point_goal;

headZ = Config.head.camOffsetZ;

function detect(color)
  local freespace = {};
  freespace.detect = 1;
  freespace.block = 0; -- whether there are cols being wholely blocked
  freespace.bound = {}; -- freespace boundary in world
  freespace.boundA = {}; -- freespace boundary in label
--[[
  freespace.a = 1; -- linear regression 
  freespace.b = 0; -- linear regression
--]]
  -- Get label handle
  labelA = Vision.labelA;

  -- Separate Frame into 8 columns : 80 x 60
  local nCol = Config.vision.freespace_scanColA; -- Search space column
  local nRow = Config.vision.freespace_scanRowA; -- Search space row

  freespace.turn = vector.zeros(nCol);
--[[
  local xBar = 0;
  local yBar = 0;
  local xyBar = 0;
  local x2Bar = 0;
--]]
  -- Search label A column by column for freespace
  for nC = 1 , nCol do
    -- Search box width
    local Xoff = labelA.m/nCol;
	local bbox = {(nC-1)*Xoff+1,nC*Xoff,1,labelA.m};
    local boundA = {(nC-0.5)*Xoff,labelA.n}
    -- stats Green
    local field = ImageProc.color_stats(labelA.data, labelA.m, 
                                        labelA.n, colorField, bbox);
	local lines = ImageProc.color_stats(labelA.data, labelA.m,
										labelA.n, colorWhite, bbox);
--	local balls = ImageProc.color_stats(labelA.data, labelA.m,
--										labelA.n, colorOrange, bbox);
	if ((field.area + lines.area)> 0) then
	  local boundary = field.axisMajor or 0; 
	  -- white compensation when white occupy the image
	  local wComThres = 0.2; -- white compensation threshold
	  if (lines.area > wComThres * field.area) then
		--print("white compensation"); 
	    boundary = boundary + lines.area/Xoff;
	  end
      boundA[2] = labelA.n - math.min(labelA.n,boundary); 
	else
	  freespace.block = 1;
    end;
    freespace.boundA[nC],freespace.boundA[nC+nCol] = boundA[1],boundA[2];

--[[	
	-- Liear Regression
	xBar = xBar + boundA[1];
	yBar = yBar + boundA[2];
	xyBar = xyBar + boundA[1]*boundA[2];
	x2Bar = x2Bar + boundA[1]^2;
--]]
    --Project to 2D coordinate
		-- TODO Find Complete Obstacle and scaling parameters
    local boundV = HeadTransform.coordinatesA(boundA,0.1); 
	-- Project to ground plane
	if (boundV[3] < -headZ) then boundV = (-headZ/boundV[3])*boundV; end
	-- Discount body offset
	uBodyOffset = mcm.get_walk_bodyOffset();
	boundV[1] = boundV[1] - uBodyOffset[1];
	boundV[2] = boundV[2] - uBodyOffset[2];

    freespace.bound[nC],freespace.bound[nC+nCol] = boundV[1],boundV[2];
  end -- end for search for columns

  -- Seek turning point
  for nC = 2 , nCol do
    local gap = freespace.boundA[nC+nCol]-freespace.boundA[nC-1+nCol];
    if (math.abs(gap) >= labelA.n/nRow) then
	  if (gap <= 0) then freespace.turn[nC] = 1;
	  else freespace.turn[nC-1] = -1; end 
	end
  end
  
  local climb = {};
  climb.idx = {};
  climb.slope = {};
  climb.num = 0;
  searchIdx = 1;
  while (searchIdx <= 39) do
    if (freespace.turn[searchIdx] == -1) and 
	   (freespace.turn[searchIdx+1] == 0) then
	  climb.num = climb.num + 1;
	  climb.idx[climb.num] = searchIdx;
	  climb.slope[climb.num] = -1;
      --print("Down"..searchIdx);
    end 
    if (freespace.turn[searchIdx+1] == 1) and
       (freespace.turn[searchIdx] == 0) then
	  climb.num = climb.num + 1;
	  climb.idx[climb.num] = searchIdx+1;
      climb.slope[climb.num] = 1;
      --print("Up"..searchIdx);
    end 
    searchIdx = searchIdx + 1;
  end

  local obst = {};
  obst.num = 0;
  obst.centroidA = {};
  obst.scale = {};
  obst.width = {};
  obst.startP = {};
  obst.endP = {};
  obst.v = {};
  obst.r = {};
  obst.a = {};
  
  sIdx = 1;
  eIdx = climb.num-1;
  if (climb.slope[1]==1) then
    sIdx = sIdx + 1;
    obst.num = obst.num+1;
    obst.startP[obst.num] = 1;
    obst.endP[obst.num] = climb.idx[1];
  end
  if (climb.slope[climb.num]==-1) then
    eIdx = eIdx - 1;
    obst.num = obst.num + 1;
    obst.startP[obst.num] = climb.idx[climb.num];
	obst.endP[obst.num] = nCol;
  end
  while (sIdx <= eIdx) do
    if (climb.slope[sIdx]==1) and (climb.slope[sIdx+1]==-1) then
      sIdx = sIdx + 1;
	else
      obst.num = obst.num + 1;
      obst.startP[obst.num] = climb.idx[sIdx];
	  obst.endP[obst.num] = climb.idx[sIdx+1];
      sIdx = sIdx + 1; 
	end
  end

--  print("climb"..climb.num,"Obstacles: "..obst.num);
  for idx = 1,obst.num do
	obst.centroidA[idx] = {}
    obst.centroidA[idx][1] = obst.startP[idx] + 
                        math.floor((obst.endP[idx]-obst.startP[idx])/2+0.5);
	obst.centroidA[idx][2] = freespace.boundA[obst.centroidA[idx][1]];
    obst.width[idx] = obst.endP[idx]-obst.startP[idx]+1;
	obst.scale[idx] = obst.width[idx]/obstSize;
    local obstV = HeadTransform.coordinatesA(obst.centroidA[idx],obst.scale[idx]); 
	-- Project to ground plane
	if (obstV[3] < -headZ) then obstV = (-headZ/obstV[3])*obstV; end
	-- Discount body offset
	uBodyOffset = mcm.get_walk_bodyOffset();
	obstV[1] = obstV[1] - uBodyOffset[1];
	obstV[2] = obstV[2] - uBodyOffset[2];
	obst.v[idx] = {};
    obst.v[idx][1],obst.v[idx][2] = obstV[1],obstV[2];
    obst.r[idx] = math.sqrt(obst.v[idx][1]^2 + obst.v[idx][2]^2);
--    print(obst.r[idx]);
  end
 
 --[[ 
  freespace.b = (xyBar - xBar*yBar)/(x2Bar-xBar^2);
  freespace.a = yBar - freespace.b*xBar;
--]]
  
  freespace.nCol = nCol;
  freespace.nRow = nRow;
  return freespace;
end
