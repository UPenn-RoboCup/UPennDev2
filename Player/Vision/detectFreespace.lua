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

  -- Separate Frame into 8 columns : 80 x 60
  local nCol = Config.vision.freespace_scanColA; -- Search space column
  local nRow = Config.vision.freespace_scanRowA; -- Search space row

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
	local ball = ImageProc.color_stats(labelA.data, labelA.m,
										labelA.n, colorOrange, bbox);
	if ((field.area + lines.area + ball.area)> 0) then
	  local boundary = field.axisMajor or 0; 
	  -- white compensation when white occupy the image
	  local wComThres = 0.2; -- white compensation threshold
	  if (lines.area > wComThres * field.area) then
		--print("white compensation"); 
	    boundary = boundary + lines.area/Xoff;
	  end
      local oComThres = 0.05; -- orange compensation threshold
      if (ball.area > oComThres * field.area) then
		--print("orange compensation");
		boundary = boundary + ball.area/Xoff;
	  end
      boundA[2] = math.floor(labelA.n - math.min(labelA.n,boundary)+1.5); 
	else
	  freespace.block = 1;
    end;
    freespace.boundA[nC],freespace.boundA[nC+nCol] = boundA[1],boundA[2];
    --Project to 2D coordinate
    local boundV = HeadTransform.rayIntersectA(boundA); 
	-- Discount body offset
	uBodyOffset = mcm.get_walk_bodyOffset();
--	print("BodyOffset"..uBodyOffset[1],uBodyOffset[2]);
	boundV[1] = boundV[1] - uBodyOffset[1];
	boundV[2] = boundV[2] - uBodyOffset[2];
    freespace.bound[nC],freespace.bound[nC+nCol] = boundV[1],boundV[2];
  end -- end for search for columns

  freespace.nCol = nCol;
  freespace.nRow = nRow;
  return freespace;
end
