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
  freespace.horizonA = {};
  -- Get horizon of image
  local h = HeadTransform.viewhorizonA();
  freespace.horizonA = h;
--  print(h[1],h[2],h[3],h[4]);
  -- Get label handle
  labelA = Vision.labelA;

  local nCol = Config.vision.freespace_scanColA; -- Search space column
  local nRow = Config.vision.freespace_scanRowA; -- Search space row
  local fillrate = 0.2;

  -- Search label A column by column for freespace
  for nC = 1 , nCol do
    -- Search box width
    local Xoff = labelA.m/nCol;
	local bbox = {(nC-1)*Xoff+1,nC*Xoff,1,labelA.n};
	if (nC==nCol) then bbox = {(nC-1)*Xoff+1,nC*Xoff-1,1,labelA.n}; end;
    local boundA = {(nC-0.5)*Xoff,labelA.n}
    -- stats Green
    local field = ImageProc.color_stats(labelA.data, labelA.m, 
                                        labelA.n, colorField, bbox);
	local lines = ImageProc.color_stats(labelA.data, labelA.m,
										labelA.n, colorWhite, bbox);
	local ball = ImageProc.color_stats(labelA.data, labelA.m,
										labelA.n, colorOrange, bbox);
	local colorAcc = field.area + lines.area + ball.area;
	if (colorAcc > 0) then
      boundA[2] = math.floor(labelA.n - colorAcc/(bbox[2]-bbox[1]+1) + 0.5); 
	else
	  freespace.block = 1;
    end;

    freespace.boundA[nC],freespace.boundA[nC+nCol] = boundA[1],boundA[2];
--  end -- end for search for columns
--[[
  -- Search label A column by column for freespace
  for nC = 1 , nCol do
    -- Search box width
    local Xoff = labelA.m/nCol;
    -- Search box height
    local Yoff = labelA.n/nRow;
    local boundA = {(nC-0.5)*Xoff,1}
	for nR=nRow,1,-1 do 
      -- search box
	  local bbox = {(nC-1)*Xoff+1,nC*Xoff,(nR-1)*Yoff+1,nR*Yoff};   
      -- stats Green
      local field = ImageProc.color_stats(labelA.data,
                             labelA.m, labelA.n, colorField, bbox);
      -- stats White -- lines are not obstacle
      local lines = ImageProc.color_stats(labelA.data,
                             labelA.m, labelA.n, colorWhite, bbox);
      -- stats orange -- ball are not obstacle
      local ball = ImageProc.color_stats(labelA.data,
                             labelA.m, labelA.n, colorOrange, bbox);
      -- filling percentage
  	  local Per = (field.area + lines.area + ball.area)/(Xoff*Yoff);      	    
	  if (Per < fillrate) then
			boundA[2] = nR*Yoff;
    	    break;
--		end
	  end -- end if (Per<=thSpace)
	end -- end for nR
	local ratio = boundA[2]/freespace.boundA[nC+nCol];
	if ratio > 0.8 then
      freespace.boundA[nC+nCol] = boundA[2];
    end
    --Project to 2D coordinate
--]]
    local boundV = HeadTransform.rayIntersectA(boundA);
    freespace.bound[nC],freespace.bound[nC+nCol] = boundV[1],boundV[2];
  end -- end for search for columns

  local horizon = HeadTransform.get_horizonA();
--  print(horizon);
  freespace.nCol = nCol;
  freespace.nRow = nRow;
  return freespace;
end
